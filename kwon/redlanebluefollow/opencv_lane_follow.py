#!/usr/bin/env python3
"""SODA AutoCar OpenCV black-tape follower (Python 3.6 compatible).

Without --drive this is preview-only and never starts the motor.
"""
from __future__ import print_function

import argparse
import os
import sys
import time

import cv2
import numpy as np

CAM_WIDTH, CAM_HEIGHT, CAM_FPS = 320, 240, 30
ROI_START_RATIO = 0.42
SIDE_CUT_RATIO = 0.075
MIN_AREA = 24.0
MAX_AREA_RATIO = 0.085
MIN_ELONGATION = 1.30
MAX_WIDTH_RATIO = 0.32
MAX_TAPE_THICKNESS_RATIO = 0.075

DRIVE_SPEED = 25
KP, KD = 0.58, 0.18
STEER_SIGN = 1.0
MAX_STEERING = 0.65
STABLE_FRAMES_TO_START = 5
LOST_FRAMES_TO_STOP = 3


def clip(value, low, high):
    return max(low, min(high, float(value)))


class LaneDetector(object):
    def __init__(self):
        self.previous_x = None

    def detect(self, frame):
        if frame is None or frame.size == 0:
            raise ValueError("empty frame")

        height, width = frame.shape[:2]
        roi_y = int(height * ROI_START_RATIO)
        roi = frame[roi_y:, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # Black tape becomes white. Otsu adjusts the cutoff for each frame.
        unused, mask = cv2.threshold(
            gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))

        # The sample camera overlay/background joins the desired tape to a
        # large dark component at the left border. Cut only that narrow border
        # strip; do not reject the remaining contour as the previous version did.
        side_cut = int(mask.shape[1] * SIDE_CUT_RATIO)
        mask[:, :side_cut] = 0
        mask[:, mask.shape[1] - side_cut:] = 0

        contour_result = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contour_result[-2]
        roi_h, roi_w = mask.shape[:2]
        max_area = roi_w * roi_h * MAX_AREA_RATIO
        max_width = roi_w * MAX_WIDTH_RATIO
        max_tape_thickness = roi_w * MAX_TAPE_THICKNESS_RATIO
        candidates = []

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < MIN_AREA or area > max_area:
                continue
            x, y, box_w, box_h = cv2.boundingRect(contour)
            # Axis-aligned boxes make a 45-degree tape look almost square.
            # A rotated box measures the actual long/short sides instead.
            rotated = cv2.minAreaRect(contour)
            rotated_w, rotated_h = rotated[1]
            short_side = max(1.0, float(min(rotated_w, rotated_h)))
            elongation = float(max(rotated_w, rotated_h)) / short_side
            if (elongation < MIN_ELONGATION or
                    short_side > max_tape_thickness or
                    box_w > max_width):
                continue
            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue
            candidates.append({
                "contour": contour,
                "area": area,
                "cx": float(moments["m10"] / moments["m00"]),
                "cy": float(moments["m01"] / moments["m00"]),
                "elongation": elongation,
            })

        reference_x = self.previous_x
        if reference_x is None:
            reference_x = roi_w * 0.5

        selected = None
        if candidates:
            def score(item):
                # Neighbouring lines are visible together. Position continuity
                # must dominate so that a long line at the screen edge cannot
                # steal the selection from the intended centre line.
                return abs(item["cx"] - reference_x) / float(roi_w)

            selected = min(candidates, key=score)
            detected_x = selected["cx"]
            if self.previous_x is None:
                self.previous_x = detected_x
            else:
                self.previous_x = self.previous_x * 0.68 + detected_x * 0.32

        target_x = roi_w * 0.5
        lane_x = self.previous_x if selected is not None else None
        error = None if lane_x is None else (lane_x - target_x) / target_x
        return {
            "found": selected is not None,
            "lane_x": lane_x,
            "error": error,
            "roi_y": roi_y,
            "mask": mask,
            "candidates": candidates,
            "selected": selected,
        }


def draw_result(frame, result, steering=None, drive=False):
    out = frame.copy()
    height, width = out.shape[:2]
    roi_y = result["roi_y"]
    cv2.line(out, (0, roi_y), (width - 1, roi_y), (255, 180, 0), 1)
    cv2.line(out, (width // 2, roi_y), (width // 2, height - 1),
             (255, 255, 0), 1)

    for item in result["candidates"]:
        contour = item["contour"].copy()
        contour[:, :, 1] += roi_y
        cv2.drawContours(out, [contour], -1, (0, 180, 255), 1)

    if result["selected"] is not None:
        cx = int(round(result["lane_x"]))
        cy = int(round(result["selected"]["cy"])) + roi_y
        cv2.circle(out, (cx, cy), 7, (0, 255, 0), -1)
        cv2.line(out, (width // 2, cy), (cx, cy), (0, 255, 0), 2)
        text = "LANE error={:+.3f}".format(result["error"])
        color = (0, 255, 0)
    else:
        text, color = "LANE LOST", (0, 0, 255)

    cv2.putText(out, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    mode = "DRIVE" if drive else "PREVIEW - MOTOR OFF"
    cv2.putText(out, mode, (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                (0, 0, 255) if drive else (255, 255, 0), 1)
    if steering is not None:
        cv2.putText(out, "steer={:+.3f}".format(steering), (8, 64),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
    return out


def process_image(image_path, output_path):
    frame = cv2.imread(image_path)
    if frame is None:
        raise RuntimeError("cannot read image: " + image_path)
    result = LaneDetector().detect(frame)
    if not cv2.imwrite(output_path, draw_result(frame, result)):
        raise RuntimeError("cannot write image: " + output_path)
    output_root, output_ext = os.path.splitext(output_path)
    if not output_ext:
        output_ext = ".jpg"
    mask_path = output_root + "_mask" + output_ext
    if not cv2.imwrite(mask_path, result["mask"]):
        raise RuntimeError("cannot write mask: " + mask_path)
    print("lane_found:", result["found"])
    print("lane_x:", result["lane_x"])
    print("normalized_error:", result["error"])
    print("candidate_count:", len(result["candidates"]))
    print("output:", output_path)
    print("mask_output:", mask_path)


def run_camera(drive):
    try:
        from pop import Pilot, Util
    except Exception as exc:
        raise RuntimeError("camera mode must run on SODA AutoCar: " + str(exc))

    Util.enable_imshow()
    pipeline = Util.gstrmer(width=CAM_WIDTH, height=CAM_HEIGHT,
                            fps=CAM_FPS, flip=0)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError("GStreamer camera could not be opened")

    car = Pilot.AutoCar()
    detector = LaneDetector()
    previous_error = 0.0
    stable_frames = 0
    lost_frames = 0

    if drive:
        print("DRIVE MODE starts in 3 seconds. Be ready to stop the car.")
        time.sleep(3.0)
    else:
        print("PREVIEW MODE: motor is off.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            result = detector.detect(frame)
            steering = 0.0

            if result["found"]:
                stable_frames += 1
                lost_frames = 0
                error = result["error"]
                steering = STEER_SIGN * (KP * error + KD * (error - previous_error))
                previous_error = error
                steering = clip(steering, -MAX_STEERING, MAX_STEERING)
                if drive and stable_frames >= STABLE_FRAMES_TO_START:
                    car.steering = steering
                    try:
                        car.forward(DRIVE_SPEED)
                    except TypeError:
                        car.forward()
            else:
                stable_frames = 0
                lost_frames += 1
                if drive and lost_frames >= LOST_FRAMES_TO_STOP:
                    car.stop()
                    car.steering = 0.0

            cv2.imshow("OpenCV lane follow",
                       draw_result(frame, result, steering, drive))
            if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        car.stop()
        car.steering = 0.0
        cap.release()
        print("Car stopped and camera released.")


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--image")
    parser.add_argument("--output", default="lane_detection_result.jpg")
    parser.add_argument("--drive", action="store_true")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])
    if args.image:
        process_image(args.image, args.output)
    else:
        run_camera(args.drive)


if __name__ == "__main__":
    main()
