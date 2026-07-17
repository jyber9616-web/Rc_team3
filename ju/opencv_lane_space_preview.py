#!/usr/bin/env python3
"""Preview two drivable spaces between three black tape boundaries.

Layout: LEFT boundary | left driving space | dashed MIDDLE boundary |
right driving space | RIGHT boundary.

This file never creates AutoCar and never starts the motor. It is a perception
preview for Python 3.6 / OpenCV 4.3 on SODA AutoCar.
"""
from __future__ import print_function

import argparse
import math
import os
import sys

import cv2
import numpy as np


CAM_WIDTH, CAM_HEIGHT, CAM_FPS = 320, 240, 30
ROI_TOP_RATIO = 0.38
ROI_BOTTOM_RATIO = 0.92

CANNY_LOW = 40
CANNY_HIGH = 120
HOUGH_THRESHOLD = 16
MIN_LINE_LENGTH = 18
MAX_LINE_GAP = 22
MIN_ANGLE_DEG = 18.0
MAX_ANGLE_DEG = 82.0
# A black tape produces two Canny/Hough edges. Short dashed segments can make
# their extrapolated bottom positions differ more than the physical tape width.
CLUSTER_GAP_PX = 36.0
TRACK_HOLD_FRAMES = 10
TRACK_SMOOTHING = 0.60
TRACK_MAX_JUMP_PX = 55.0

# Calibrated from red_tape_web_test.py on the actual SODA camera.
# Distant tape becomes pale orange under the SODA camera's auto white balance.
# These values still reject the low-saturation beige floor while retaining the
# thin outer boundaries seen in the real WebView frames.
RED_HUE_WIDTH = 20
# New red insulation tape can look pale under the SODA camera's automatic
# exposure.  30 retains that tape while RED_VALUE_MIN still rejects the dark
# floor seams; 45 was unnecessarily strict on the altered course.
RED_SATURATION_MIN = 30
RED_VALUE_MIN = 45

BOUNDARY_COLORS = {
    "left": (255, 80, 40),
    "middle": (0, 255, 255),
    "right": (40, 80, 255),
}


def clip(value, low, high):
    return max(low, min(high, float(value)))


def x_at_y(x1, y1, x2, y2, target_y):
    dy = float(y2 - y1)
    if abs(dy) < 1.0:
        return None
    return float(x1) + (float(target_y - y1) * float(x2 - x1) / dy)


class MultiLaneDetector(object):
    def __init__(self, current_lane="right", detection_mode="dark"):
        self.current_lane = current_lane
        self.detection_mode = detection_mode
        self.allow_fast_motion = False
        self.tracks = {}
        self.missing = {"left": 0, "middle": 0, "right": 0}

    def _extract_line_models(self, edges, min_angle=MIN_ANGLE_DEG,
                             max_angle=MAX_ANGLE_DEG):
        roi_h, roi_w = edges.shape[:2]
        near_y = int(roi_h * 0.82)
        far_y = int(roi_h * 0.22)
        hough = cv2.HoughLinesP(
            edges,
            1,
            np.pi / 180.0,
            HOUGH_THRESHOLD,
            minLineLength=MIN_LINE_LENGTH,
            maxLineGap=MAX_LINE_GAP,
        )
        models = []
        if hough is None:
            return models, near_y, far_y

        for raw in hough[:, 0, :]:
            x1, y1, x2, y2 = [int(value) for value in raw]
            dx = float(x2 - x1)
            dy = float(y2 - y1)
            length = math.sqrt(dx * dx + dy * dy)
            angle = abs(math.degrees(math.atan2(dy, dx)))
            if angle > 90.0:
                angle = 180.0 - angle
            if angle < min_angle or angle > max_angle:
                continue

            near_x = x_at_y(x1, y1, x2, y2, near_y)
            far_x = x_at_y(x1, y1, x2, y2, far_y)
            if near_x is None or far_x is None:
                continue
            if near_x < -0.35 * roi_w or near_x > 1.35 * roi_w:
                continue
            if far_x < -0.55 * roi_w or far_x > 1.55 * roi_w:
                continue

            models.append({
                "segment": (x1, y1, x2, y2),
                "near_x": near_x,
                "far_x": far_x,
                "length": length,
            })
        return models, near_y, far_y

    @staticmethod
    def _cluster_models(models):
        if not models:
            return []
        ordered = sorted(models, key=lambda item: item["near_x"])
        groups = []
        for model in ordered:
            if not groups:
                groups.append([model])
                continue
            last = groups[-1]
            weight = sum(item["length"] for item in last)
            centre = sum(item["near_x"] * item["length"] for item in last) / weight
            if abs(model["near_x"] - centre) <= CLUSTER_GAP_PX:
                last.append(model)
            else:
                groups.append([model])

        clusters = []
        for group in groups:
            weight = sum(item["length"] for item in group)
            clusters.append({
                "near_x": sum(item["near_x"] * item["length"] for item in group) / weight,
                "far_x": sum(item["far_x"] * item["length"] for item in group) / weight,
                "weight": weight,
                "segments": [item["segment"] for item in group],
            })
        return clusters

    def _label_clusters(self, clusters):
        # Extra floor/wall edges and the two sides of one tape can appear as
        # separate clusters. Use only the two boundaries adjacent to the lane
        # where the camera currently sits. The opposite outer boundary is
        # estimated later from the observed lane width.
        strongest = sorted(clusters, key=lambda item: item["weight"], reverse=True)[:3]
        strongest = sorted(strongest, key=lambda item: item["near_x"])
        if len(strongest) >= 2:
            if self.current_lane == "right":
                return {"middle": strongest[-2], "right": strongest[-1]}
            return {"left": strongest[0], "middle": strongest[1]}
        return {}

    def _update_tracks(self, observed):
        for name in ("left", "middle", "right"):
            if name in observed:
                item = observed[name]
                if name in self.tracks:
                    old = self.tracks[name]
                    # Ignore a single-frame label swap or reflection. Real
                    # boundary motion between adjacent camera frames is small.
                    max_jump = (150.0 if self.allow_fast_motion
                                else TRACK_MAX_JUMP_PX)
                    if abs(item["near_x"] - old["near_x"]) > max_jump:
                        self.missing[name] += 1
                        if self.missing[name] > TRACK_HOLD_FRAMES:
                            self.tracks.pop(name, None)
                        continue
                    alpha = TRACK_SMOOTHING
                    item = dict(item)
                    item["near_x"] = old["near_x"] * (1.0 - alpha) + item["near_x"] * alpha
                    item["far_x"] = old["far_x"] * (1.0 - alpha) + item["far_x"] * alpha
                self.tracks[name] = item
                self.missing[name] = 0
            else:
                self.missing[name] += 1
                if self.missing[name] > TRACK_HOLD_FRAMES:
                    self.tracks.pop(name, None)

    def detect(self, frame):
        if frame is None or frame.size == 0:
            raise ValueError("empty frame")
        height, width = frame.shape[:2]
        roi_top = int(height * ROI_TOP_RATIO)
        roi_bottom = int(height * ROI_BOTTOM_RATIO)
        roi = frame[roi_top:roi_bottom, :]
        if self.detection_mode == "red":
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            low_red = cv2.inRange(
                hsv,
                np.array([0, RED_SATURATION_MIN, RED_VALUE_MIN], dtype=np.uint8),
                np.array([RED_HUE_WIDTH, 255, 255], dtype=np.uint8),
            )
            high_red = cv2.inRange(
                hsv,
                np.array([180 - RED_HUE_WIDTH, RED_SATURATION_MIN,
                          RED_VALUE_MIN], dtype=np.uint8),
                np.array([179, 255, 255], dtype=np.uint8),
            )
            color_mask = cv2.bitwise_or(low_red, high_red)
            color_mask = cv2.morphologyEx(
                color_mask, cv2.MORPH_OPEN,
                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
            color_mask = cv2.morphologyEx(
                color_mask, cv2.MORPH_CLOSE,
                cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7)))
            edges = cv2.Canny(color_mask, 40, 120)
        else:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(gray, CANNY_LOW, CANNY_HIGH)

        # Remove the image frame itself; it is never a road boundary.
        edges[:, :3] = 0
        edges[:, edges.shape[1] - 3:] = 0
        if self.detection_mode == "red":
            # Perspective makes the distant outer boundaries almost
            # horizontal and a centred divider almost vertical.
            models, near_y, far_y = self._extract_line_models(
                edges, min_angle=9.0, max_angle=89.0)
        else:
            models, near_y, far_y = self._extract_line_models(edges)
        clusters = self._cluster_models(models)
        observed = self._label_clusters(clusters)
        self._update_tracks(observed)
        return {
            "roi_top": roi_top,
            "roi_bottom": roi_bottom,
            "edges": edges,
            "models": models,
            "clusters": clusters,
            "boundaries": dict(self.tracks),
            "observed_boundaries": dict(observed),
            "near_y": near_y,
            "far_y": far_y,
            "detection_mode": self.detection_mode,
        }


def complete_boundaries(result):
    """Estimate one off-screen outer boundary assuming equal lane widths."""
    boundaries = dict(result["boundaries"])
    estimated = set()
    if ("left" not in boundaries and
            "middle" in boundaries and "right" in boundaries):
        middle = boundaries["middle"]
        right = boundaries["right"]
        boundaries["left"] = {
            "near_x": 2.0 * middle["near_x"] - right["near_x"],
            "far_x": 2.0 * middle["far_x"] - right["far_x"],
            "weight": 0.0,
            "segments": [],
        }
        estimated.add("left")
    if ("right" not in boundaries and
            "left" in boundaries and "middle" in boundaries):
        left = boundaries["left"]
        middle = boundaries["middle"]
        boundaries["right"] = {
            "near_x": 2.0 * middle["near_x"] - left["near_x"],
            "far_x": 2.0 * middle["far_x"] - left["far_x"],
            "weight": 0.0,
            "segments": [],
        }
        estimated.add("right")
    return boundaries, estimated


def lane_boundaries(result, lane_name):
    boundaries, estimated = complete_boundaries(result)
    if lane_name == "left":
        names = ("left", "middle")
    else:
        names = ("middle", "right")
    if names[0] not in boundaries or names[1] not in boundaries:
        return None
    return boundaries[names[0]], boundaries[names[1]]


def draw_dashed_line(image, point1, point2, color, thickness=2, pieces=10):
    x1, y1 = point1
    x2, y2 = point2
    for index in range(pieces):
        if index % 2 != 0:
            continue
        start_ratio = float(index) / pieces
        end_ratio = float(index + 1) / pieces
        start = (
            int(round(x1 + (x2 - x1) * start_ratio)),
            int(round(y1 + (y2 - y1) * start_ratio)),
        )
        end = (
            int(round(x1 + (x2 - x1) * end_ratio)),
            int(round(y1 + (y2 - y1) * end_ratio)),
        )
        cv2.line(image, start, end, color, thickness)


def transition_path(result, current_lane, target_lane):
    """Return the centre path of the requested lane.

    During a lane change, stopping just beyond MIDDLE leaves the RC car
    pointed at the divider.  Aim at the estimated centre of the whole target
    lane instead, then let the normal lane-centering controller take over.
    """
    boundaries, unused = complete_boundaries(result)
    if "middle" not in boundaries:
        return None
    if current_lane == target_lane:
        pair = lane_boundaries(result, target_lane)
        if pair is None:
            return None
        first, second = pair
        return {
            "near_x": (first["near_x"] + second["near_x"]) * 0.5,
            "far_x": (first["far_x"] + second["far_x"]) * 0.5,
        }

    middle = boundaries["middle"]
    if current_lane == "right" and target_lane == "left":
        if "right" not in boundaries:
            return None
        width_near = boundaries["right"]["near_x"] - middle["near_x"]
        width_far = boundaries["right"]["far_x"] - middle["far_x"]
        sign = -1.0
    elif current_lane == "left" and target_lane == "right":
        if "left" not in boundaries:
            return None
        width_near = middle["near_x"] - boundaries["left"]["near_x"]
        width_far = middle["far_x"] - boundaries["left"]["far_x"]
        sign = 1.0
    else:
        return None

    # One half lane width from MIDDLE is the centre of the adjacent lane.
    cross_fraction = 0.50
    return {
        "near_x": middle["near_x"] + sign * width_near * cross_fraction,
        "far_x": middle["far_x"] + sign * width_far * cross_fraction,
    }


def draw_preview(frame, result, target_lane, current_lane,
                 lookahead_ratio=0.45):
    out = frame.copy()
    overlay = out.copy()
    roi_top = result["roi_top"]
    near_y = result["near_y"] + roi_top
    far_y = result["far_y"] + roi_top

    # Raw accepted Hough segments are intentionally thin and dim.
    for model in result["models"]:
        x1, y1, x2, y2 = model["segment"]
        cv2.line(out, (x1, y1 + roi_top), (x2, y2 + roi_top), (120, 120, 120), 1)

    completed, estimated_names = complete_boundaries(result)
    for name in ("left", "middle", "right"):
        if name not in estimated_names:
            continue
        boundary = completed[name]
        p_far = (int(round(boundary["far_x"])), far_y)
        p_near = (int(round(boundary["near_x"])), near_y)
        draw_dashed_line(out, p_far, p_near, (255, 0, 255), 2)
        cv2.putText(out, name.upper() + " EST", (p_near[0] - 28, p_near[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 0, 255), 1)

    for name in ("left", "middle", "right"):
        boundary = result["boundaries"].get(name)
        if boundary is None:
            continue
        color = BOUNDARY_COLORS[name]
        p_far = (int(round(boundary["far_x"])), far_y)
        p_near = (int(round(boundary["near_x"])), near_y)
        cv2.line(out, p_far, p_near, color, 3)
        cv2.circle(out, p_near, 5, color, -1)
        cv2.putText(out, name.upper(), (p_near[0] - 22, p_near[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)

    pair = lane_boundaries(result, target_lane)
    target_x = None
    if pair is not None:
        first, second = pair
        polygon = np.array([
            [int(round(first["far_x"])), far_y],
            [int(round(second["far_x"])), far_y],
            [int(round(second["near_x"])), near_y],
            [int(round(first["near_x"])), near_y],
        ], dtype=np.int32)
        fill_color = (40, 170, 40) if target_lane == "left" else (180, 90, 20)
        cv2.fillPoly(overlay, [polygon], fill_color)
        out = cv2.addWeighted(overlay, 0.22, out, 0.78, 0.0)

        target_near_x = (first["near_x"] + second["near_x"]) * 0.5
        target_far_x = (first["far_x"] + second["far_x"]) * 0.5
        target_x = target_near_x
        cv2.line(out,
                 (int(round(target_far_x)), far_y),
                 (int(round(target_near_x)), near_y),
                 (0, 255, 0), 4)
        cv2.circle(out, (int(round(target_near_x)), near_y), 7, (0, 255, 0), -1)

    found_names = [name for name in ("left", "middle", "right")
                   if name in result["boundaries"]]
    status = "BOUNDARIES: " + ",".join(found_names).upper()
    cv2.putText(out, status, (7, 20), cv2.FONT_HERSHEY_SIMPLEX,
                0.48, (255, 255, 255), 2)
    if estimated_names:
        estimated_text = "ESTIMATED: " + ",".join(sorted(estimated_names)).upper()
        cv2.putText(out, estimated_text, (7, 78), cv2.FONT_HERSHEY_SIMPLEX,
                    0.40, (255, 0, 255), 1)
    cv2.putText(out, "TARGET SPACE: " + target_lane.upper(), (7, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                (0, 255, 0) if target_x is not None else (0, 0, 255), 2)
    if target_x is None:
        cv2.putText(out, "TARGET UNAVAILABLE", (7, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 255), 2)
    else:
        error = (target_x - frame.shape[1] * 0.5) / (frame.shape[1] * 0.5)
        cv2.putText(out, "space_error={:+.3f}".format(error), (7, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 0), 2)

    planned = transition_path(result, current_lane, target_lane)
    if planned is not None:
        plan_far = (int(round(planned["far_x"])), far_y)
        plan_near = (int(round(planned["near_x"])), near_y)
        cv2.line(out, plan_far, plan_near, (255, 255, 0), 3)
        # Steering should use a forward look-ahead point, not the near point
        # which may be outside the image at the start of a lane change.
        look_ratio = lookahead_ratio
        look_x = planned["far_x"] + (planned["near_x"] - planned["far_x"]) * look_ratio
        look_y = int(round(far_y + (near_y - far_y) * look_ratio))
        cv2.circle(out, (int(round(look_x)), look_y), 7, (255, 255, 0), -1)
        guidance_error = (look_x - frame.shape[1] * 0.5) / (frame.shape[1] * 0.5)
        cv2.putText(out, "guide_error={:+.3f}".format(guidance_error), (7, 96),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 0), 1)
    return out


def process_image(image_path, output_path, current_lane, target_lane):
    frame = cv2.imread(image_path)
    if frame is None:
        raise RuntimeError("cannot read image: " + image_path)
    detector = MultiLaneDetector(current_lane=current_lane)
    result = detector.detect(frame)
    preview = draw_preview(frame, result, target_lane, current_lane)
    if not cv2.imwrite(output_path, preview):
        raise RuntimeError("cannot write: " + output_path)
    root, ext = os.path.splitext(output_path)
    if not ext:
        ext = ".jpg"
    edge_path = root + "_edges" + ext
    cv2.imwrite(edge_path, result["edges"])
    print("boundaries:", sorted(result["boundaries"].keys()))
    print("raw_line_count:", len(result["models"]))
    print("cluster_count:", len(result["clusters"]))
    print("target_lane:", target_lane)
    print("target_available:", lane_boundaries(result, target_lane) is not None)
    unused, estimated = complete_boundaries(result)
    print("estimated_boundaries:", sorted(estimated))
    print("output:", output_path)
    print("edges_output:", edge_path)


def run_camera(current_lane, initial_target):
    try:
        from pop import Util
    except Exception as exc:
        raise RuntimeError("camera mode must run on SODA AutoCar: " + str(exc))
    Util.enable_imshow()
    pipeline = Util.gstrmer(width=CAM_WIDTH, height=CAM_HEIGHT,
                            fps=CAM_FPS, flip=0)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError("GStreamer camera could not be opened")

    detector = MultiLaneDetector(current_lane=current_lane)
    target_lane = initial_target
    print("Preview only: this file cannot start the motor.")
    print("Press A for left space, D for right space, Q to quit.")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            result = detector.detect(frame)
            cv2.imshow("Two-lane space preview",
                       draw_preview(frame, result, target_lane, current_lane))
            key = cv2.waitKey(1) & 0xFF
            if key == ord("a"):
                target_lane = "left"
            elif key == ord("d"):
                target_lane = "right"
            elif key in (ord("q"), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        print("Camera released.")


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--image")
    parser.add_argument("--output", default="lane_space_result.jpg")
    parser.add_argument("--current-lane", choices=("left", "right"), default="right")
    parser.add_argument("--target-lane", choices=("left", "right"), default="right")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])
    if args.image:
        process_image(args.image, args.output, args.current_lane, args.target_lane)
    else:
        run_camera(args.current_lane, args.target_lane)


if __name__ == "__main__":
    main()
