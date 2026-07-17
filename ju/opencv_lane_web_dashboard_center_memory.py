#!/usr/bin/env python3
"""Right-to-left lane change dashboard with remembered centre-divider guidance.

This is intentionally a separate entry point.  It keeps
``opencv_lane_web_dashboard.py`` unchanged.

Problem addressed
---------------
When the RC car leaves the right lane, the old dashboard immediately changes
the meaning of the strongest Hough-line clusters.  On a dashed centre line
that can make the car lose the divider exactly while it must cross it.

This version records the visible MIDDLE and RIGHT boundaries at the instant a
right-to-left change is requested.  Their separation is the measured lane
width.  Until a real LEFT + MIDDLE pair has been stable, it:

* follows only the remembered centre-divider (the divider is expected to move
  to the *right* of the camera as the car enters the left lane), and
* targets one half of the saved lane width to the left of that divider.

It is a guarded forward-drive test for the SODA/Python 3.6 environment, not a
general road-driving system.  The original dashboard remains available as a
fallback.
"""
from __future__ import print_function

import sys
import time
import types

import cv2
from flask import jsonify

import opencv_lane_web_dashboard as base


# A divider can disappear during a gap in the dashed tape.  Keep the last
# geometrically matched divider only for this short interval; it cannot keep
# the car driving indefinitely if the camera no longer sees the tape.
DIVIDER_HOLD_SECONDS = 0.40
DIVIDER_MAX_STEP_PX = 80.0
LEFT_ACQUIRE_FRAMES = 4
START_REFERENCE_FRAMES = 6
START_REFERENCE_TOLERANCE_PX = 20.0
# The old switch used five labelled frames.  That is not enough: the detector
# can consistently label two wrong red edges for five frames.  Require ten
# consecutive frames that also agree with the remembered x(y) geometry.
NORMAL_LEFT_PAIR_FRAMES = 10
MEMORY_Y_SAMPLES = (0.20, 0.50, 0.82)
MEMORY_Y_MATCH_TOLERANCE_PX = 22.0
CHANGE_TIMEOUT_SECONDS = 12.0
CROSS_TO_RIGHT_RATIO = 0.08


def _line_copy(line):
    """Copy only the line geometry needed by control and preview."""
    return {
        "near_x": float(line["near_x"]),
        "far_x": float(line["far_x"]),
        "weight": float(line.get("weight", 0.0)),
        "segments": [],
    }


def _line_shift(line, near_delta, far_delta):
    shifted = _line_copy(line)
    shifted["near_x"] += float(near_delta)
    shifted["far_x"] += float(far_delta)
    shifted["weight"] = 0.0
    return shifted


def _blend_line(old, new, alpha=0.70):
    out = _line_copy(new)
    out["near_x"] = old["near_x"] * (1.0 - alpha) + new["near_x"] * alpha
    out["far_x"] = old["far_x"] * (1.0 - alpha) + new["far_x"] * alpha
    return out


class CentreDividerMemory(object):
    """Track one physical divider independently from the detector labels."""

    def __init__(self):
        self.start_divider = None
        self.start_width_near = None
        self.start_width_far = None
        self.start_reference_frames = 0
        self.start_reference_locked = False
        self.start_reference_note = "waiting for middle/right"
        self.reset()

    def reset(self):
        """Clear one manoeuvre but preserve a deliberately locked start line."""
        self.stage = "idle"
        self.change_started_at = None
        self.divider = None
        self.width_near = None
        self.width_far = None
        self.left_outer = None
        self.last_seen_at = None
        self.age_frames = 0
        self.left_seen_frames = 0
        self.normal_pair_frames = 0
        self.y_match_error_px = None
        self.velocity_near = 0.0
        self.velocity_far = 0.0
        self.note = "idle"

    def clear_start_reference(self):
        self.start_divider = None
        self.start_width_near = None
        self.start_width_far = None
        self.start_reference_frames = 0
        self.start_reference_locked = False
        self.start_reference_note = "waiting for middle/right"

    def observe_start_reference(self, result, frame_width, now):
        """Build a stable MIDDLE/RIGHT reference while the car is stopped.

        It is intentionally updated only before Drive is pressed.  The
        eventual lane-change reference is therefore the divider that was
        actually visible at departure, not a line relabelled during motion.
        """
        if self.start_reference_locked:
            return self.start_reference_frames >= START_REFERENCE_FRAMES
        boundaries = result.get("boundaries", {})
        middle = boundaries.get("middle")
        right = boundaries.get("right")
        if middle is None or right is None:
            self.start_reference_frames = 0
            self.start_reference_note = "waiting for middle/right"
            return False
        width_near = right["near_x"] - middle["near_x"]
        width_far = right["far_x"] - middle["far_x"]
        if not self._valid_width(width_near, width_far, frame_width):
            self.start_reference_frames = 0
            self.start_reference_note = "rejected start-line width"
            return False

        candidate = _line_copy(middle)
        same_line = False
        if self.start_divider is not None:
            errors = []
            for ratio in MEMORY_Y_SAMPLES:
                errors.append(abs(_x_at_y_ratio(candidate, ratio) -
                                  _x_at_y_ratio(self.start_divider, ratio)))
            same_line = (max(errors) <= START_REFERENCE_TOLERANCE_PX and
                         abs(width_near - self.start_width_near) <= 28.0 and
                         abs(width_far - self.start_width_far) <= 28.0)
        if same_line:
            self.start_divider = _blend_line(self.start_divider, candidate,
                                              alpha=0.45)
            self.start_width_near = (self.start_width_near * 0.55 +
                                     width_near * 0.45)
            self.start_width_far = (self.start_width_far * 0.55 +
                                    width_far * 0.45)
            self.start_reference_frames += 1
        else:
            self.start_divider = candidate
            self.start_width_near = float(width_near)
            self.start_width_far = float(width_far)
            self.start_reference_frames = 1
        self.start_reference_note = "checking start divider ({0}/{1})".format(
            self.start_reference_frames, START_REFERENCE_FRAMES)
        return self.start_reference_frames >= START_REFERENCE_FRAMES

    def lock_start_reference(self):
        if (self.start_divider is None or
                self.start_reference_frames < START_REFERENCE_FRAMES):
            return False
        self.start_reference_locked = True
        self.start_reference_note = "locked at drive start"
        return True

    def arm(self, started_at):
        self.reset()
        self.change_started_at = started_at
        if (self.start_reference_locked and self.start_divider is not None):
            self.divider = _line_copy(self.start_divider)
            self.width_near = float(self.start_width_near)
            self.width_far = float(self.start_width_far)
            self.last_seen_at = time.time()
            self.stage = "transfer"
            self.note = "using middle/right saved at drive start"
        else:
            self.stage = "arming"
            self.note = "waiting for stable middle/right snapshot"

    def _valid_width(self, near, far, frame_width):
        # These broad bounds reject a label swap but allow the perspective
        # width seen by the 320 px SODA camera.
        return (30.0 <= near <= frame_width * 0.90 and
                12.0 <= far <= frame_width * 0.90)

    def start_from_result(self, result, frame_width, now):
        """Freeze the right-lane MIDDLE-to-RIGHT spacing once."""
        boundaries = result.get("boundaries", {})
        middle = boundaries.get("middle")
        right = boundaries.get("right")
        if middle is None or right is None:
            self.note = "middle/right snapshot not available"
            return False

        width_near = right["near_x"] - middle["near_x"]
        width_far = right["far_x"] - middle["far_x"]
        if not self._valid_width(width_near, width_far, frame_width):
            self.note = "rejected implausible middle/right width"
            return False

        self.divider = _line_copy(middle)
        self.width_near = float(width_near)
        self.width_far = float(width_far)
        self.last_seen_at = now
        self.stage = "transfer"
        self.note = "saved divider and right-lane width"
        return True

    def _select_divider_candidate(self, clusters, frame_width):
        if self.divider is None:
            return None
        expected_near = self.divider["near_x"] + self.velocity_near
        expected_far = self.divider["far_x"] + self.velocity_far
        old_slope = self.divider["far_x"] - self.divider["near_x"]
        best = None
        best_score = None
        for candidate in clusters:
            near_x = candidate["near_x"]
            far_x = candidate["far_x"]
            if near_x < 4.0 or near_x > frame_width - 4.0:
                continue
            near_step = near_x - self.divider["near_x"]
            far_step = far_x - self.divider["far_x"]
            # The centre divider moves to image-right on this manoeuvre.  A
            # small backwards movement is allowed for vibration, but a large
            # jump would normally mean that another red line was selected.
            if near_step < -32.0 or near_step > DIVIDER_MAX_STEP_PX:
                continue
            if abs(far_step) > DIVIDER_MAX_STEP_PX * 1.25:
                continue
            slope = far_x - near_x
            score = (abs(near_x - expected_near) +
                     0.60 * abs(far_x - expected_far) +
                     0.25 * abs(slope - old_slope))
            if near_step < -8.0:
                score += 22.0
            # Prefer a real long tape segment when geometry is otherwise
            # similar, but never let length override geometry by itself.
            score -= min(float(candidate.get("weight", 0.0)), 80.0) * 0.04
            if best_score is None or score < best_score:
                best = candidate
                best_score = score
        return best

    def update_divider(self, result, frame_width, now):
        """Update from raw clusters, or use a very short dash-gap hold."""
        if self.divider is None:
            return False
        candidate = self._select_divider_candidate(
            result.get("clusters", []), frame_width)
        if candidate is not None:
            previous = self.divider
            measured = _line_copy(candidate)
            self.divider = _blend_line(previous, measured)
            self.velocity_near = max(-18.0, min(
                18.0, self.divider["near_x"] - previous["near_x"]))
            self.velocity_far = max(-18.0, min(
                18.0, self.divider["far_x"] - previous["far_x"]))
            self.last_seen_at = now
            self.age_frames += 1
            return True
        return (self.last_seen_at is not None and
                now - self.last_seen_at <= DIVIDER_HOLD_SECONDS)

    def update_left_outer(self, result):
        """Find an outer line at one saved lane width left of the divider."""
        if self.divider is None or self.width_near is None:
            self.left_seen_frames = 0
            return None
        expected_near = self.divider["near_x"] - self.width_near
        expected_far = self.divider["far_x"] - self.width_far
        limit = max(48.0, self.width_near * 0.75)
        best = None
        best_score = None
        for candidate in result.get("clusters", []):
            # It must be clearly on the left side of the tracked divider.
            if candidate["near_x"] >= (self.divider["near_x"] -
                                        self.width_near * 0.28):
                continue
            score = (abs(candidate["near_x"] - expected_near) +
                     0.65 * abs(candidate["far_x"] - expected_far))
            if score > limit:
                continue
            if best_score is None or score < best_score:
                best = candidate
                best_score = score
        if best is None:
            # The outer red boundary is solid, but blur can still remove it
            # for one frame.  Decay instead of erasing the y-reference.
            self.left_seen_frames = max(0, self.left_seen_frames - 1)
            return None

        if self.left_outer is None:
            self.left_outer = _line_copy(best)
        else:
            self.left_outer = _blend_line(self.left_outer, best)
        self.left_seen_frames += 1
        return self.left_outer

    def guide(self):
        """Centre of the target left lane, measured from saved divider width."""
        if self.divider is None or self.width_near is None:
            return None
        return {
            "near_x": self.divider["near_x"] - self.width_near * 0.5,
            "far_x": self.divider["far_x"] - self.width_far * 0.5,
        }

    def crossed_to_right_of_camera(self, frame_width):
        return (self.divider is not None and
                self.divider["near_x"] >= frame_width *
                (0.5 + CROSS_TO_RIGHT_RATIO))

    def virtual_boundaries(self):
        """Supply a consistent LEFT/MIDDLE/RIGHT geometry to the preview."""
        if self.divider is None or self.width_near is None:
            return None
        middle = _line_copy(self.divider)
        if self.left_outer is not None:
            left = _line_copy(self.left_outer)
        else:
            left = _line_shift(middle, -self.width_near, -self.width_far)
        right = _line_shift(middle, self.width_near, self.width_far)
        return {"left": left, "middle": middle, "right": right}

    def valid(self, now):
        return (self.divider is not None and self.last_seen_at is not None and
                now - self.last_seen_at <= DIVIDER_HOLD_SECONDS)


memory = CentreDividerMemory()


def _memory_preview_result(result):
    virtual = memory.virtual_boundaries()
    if virtual is None:
        return result
    preview_result = dict(result)
    preview_result["boundaries"] = virtual
    return preview_result


def _draw_memory_overlay(image, frame_width):
    if memory.divider is None:
        return image
    out = image
    roi_top = int(out.shape[0] * 0.38)
    roi_height = int(out.shape[0] * (0.92 - 0.38))
    far_y = roi_top + int(roi_height * 0.22)
    near_y = roi_top + int(roi_height * 0.82)
    divider_far = (int(round(memory.divider["far_x"])), far_y)
    divider_near = (int(round(memory.divider["near_x"])), near_y)
    cv2.line(out, divider_far, divider_near, (255, 0, 255), 3)
    guide = memory.guide()
    if guide is not None:
        guide_far = (int(round(guide["far_x"])), far_y)
        guide_near = (int(round(guide["near_x"])), near_y)
        cv2.line(out, guide_far, guide_near, (255, 255, 0), 3)
    message = "DIVIDER MEMORY: " + memory.stage.upper()
    cv2.putText(out, message, (7, 98), cv2.FONT_HERSHEY_SIMPLEX,
                0.40, (255, 0, 255), 1)
    if memory.width_near is not None:
        cv2.putText(out, "saved_width={0:.0f}".format(memory.width_near),
                    (7, 115), cv2.FONT_HERSHEY_SIMPLEX,
                    0.40, (255, 255, 0), 1)
    match_text = "x(y) match={0}/{1}".format(
        memory.normal_pair_frames, NORMAL_LEFT_PAIR_FRAMES)
    if memory.y_match_error_px is not None:
        match_text += " err={0:.0f}px".format(memory.y_match_error_px)
    cv2.putText(out, match_text, (7, 132), cv2.FONT_HERSHEY_SIMPLEX,
                0.40, (255, 255, 0), 1)
    return out


def _pair_is_plausible(pair):
    if pair is None or memory.width_near is None:
        return False
    first, second = pair
    near_width = second["near_x"] - first["near_x"]
    far_width = second["far_x"] - first["far_x"]
    return (near_width > memory.width_near * 0.38 and
            near_width < memory.width_near * 1.75 and
            far_width > memory.width_far * 0.30 and
            far_width < memory.width_far * 1.90)


def _x_at_y_ratio(line, ratio):
    """x(y) on the detected line: 0.0 is far, 1.0 is near camera."""
    return (line["far_x"] + (line["near_x"] - line["far_x"]) * ratio)


def _pair_matches_y_memory(pair):
    """Accept LEFT/MIDDLE only if both agree with saved lines over y.

    A perspective lane boundary is represented by x(y), not by one bottom
    x-coordinate.  Checking far, middle and near y positions rejects an
    accidental Hough edge that happens to cross the expected position once.
    """
    reference = memory.virtual_boundaries()
    if (pair is None or reference is None or memory.left_outer is None or
            memory.left_seen_frames < LEFT_ACQUIRE_FRAMES or
            not _pair_is_plausible(pair)):
        memory.y_match_error_px = None
        return False

    actual_left, actual_middle = pair
    expected_left = reference["left"]
    expected_middle = reference["middle"]
    errors = []
    for ratio in MEMORY_Y_SAMPLES:
        errors.append(abs(_x_at_y_ratio(actual_left, ratio) -
                          _x_at_y_ratio(expected_left, ratio)))
        errors.append(abs(_x_at_y_ratio(actual_middle, ratio) -
                          _x_at_y_ratio(expected_middle, ratio)))
    memory.y_match_error_px = max(errors)
    return memory.y_match_error_px <= MEMORY_Y_MATCH_TOLERANCE_PX


def _stop_for_loss(reason):
    with base.control_lock:
        with base.state_lock:
            base.state["drive_enabled"] = False
            base.state["steering_enabled"] = False
            base.state["lane_change_active"] = False
            base.state["lane_change_direction"] = None
            base.state["lane_change_started_at"] = None
            base.state["lane_change_settle_until"] = None
            base.state["speed_cmd"] = 0
            base.state["last_stop_reason"] = reason
        if base.steering_hardware is not None:
            base.steering_hardware.stop()


def _finish_lane_change(service):
    """Commit only after the tracked divider has crossed and LEFT is stable."""
    memory.stage = "acquire-left"
    service.detector.current_lane = "left"
    virtual = memory.virtual_boundaries()
    if virtual is not None:
        # Do not throw away the divider exactly at the state transition.
        service.detector.tracks = {
            "left": _line_copy(virtual["left"]),
            "middle": _line_copy(virtual["middle"]),
        }
        service.detector.missing = {"left": 0, "middle": 0, "right": 0}
    service.drive_missing_frames = 0
    service.previous_guide_error = 0.0
    with base.state_lock:
        if base.state["lane_change_active"]:
            base.state["current_lane"] = "left"
            base.state["target_lane"] = "left"
            base.state["lane_change_active"] = False
            base.state["lane_change_direction"] = None
            base.state["lane_change_started_at"] = None
            base.state["lane_change_settle_until"] = time.time() + 1.0


def _process_loop_with_divider_memory(self):
    """Replacement process loop; capture and all Flask routes stay original."""
    captured_frames = 0
    last_raw_sequence = 0
    publish_every = max(1, int(round(float(base.CAM_FPS) / base.STREAM_FPS)))

    while not base.stop_event.is_set():
        with self.raw_condition:
            self.raw_condition.wait_for(
                lambda: (self.raw_sequence != last_raw_sequence or
                         base.stop_event.is_set()), timeout=1.0)
            if base.stop_event.is_set():
                break
            if self.latest_frame is None:
                continue
            frame = self.latest_frame
            last_raw_sequence = self.raw_sequence

        captured_frames += 1
        publish_frame = captured_frames % publish_every == 0
        now = time.time()
        with base.state_lock:
            current_lane = base.state["current_lane"]
            target_lane = base.state["target_lane"]
            steering_enabled = base.state["steering_enabled"]
            steer_sign = base.state["steer_sign"]
            drive_enabled = base.state["drive_enabled"]
            configured_speed = base.state["configured_speed"]
            lane_change_active = base.state["lane_change_active"]
            lane_change_started_at = base.state["lane_change_started_at"]

        self.detector.allow_fast_motion = lane_change_active
        if self.detector.current_lane != current_lane:
            self.detector.current_lane = current_lane
            self.detector.tracks = {}
            self.detector.missing = {"left": 0, "middle": 0, "right": 0}

        if lane_change_active and lane_change_started_at != memory.change_started_at:
            memory.arm(lane_change_started_at)
        elif (not lane_change_active and current_lane == "right" and
              memory.stage != "idle"):
            memory.reset()

        try:
            result = self.detector.detect(frame)
            control_error = None
            frame_width = frame.shape[1]
            memory_mode = False

            # Before drive starts, make a multi-frame reference of the real
            # divider.  It is frozen by /api/drive and survives the later
            # label changes that occur while crossing into the left lane.
            if (not drive_enabled and not lane_change_active and
                    current_lane == "right" and target_lane == "right"):
                memory.observe_start_reference(result, frame_width, now)

            if lane_change_active:
                elapsed = now - lane_change_started_at
                if elapsed > CHANGE_TIMEOUT_SECONDS:
                    _stop_for_loss("divider-memory lane change timeout")
                    drive_enabled = False
                    steering_enabled = False
                    lane_change_active = False
                    control_error = "divider-memory lane change timeout; motor stopped"
                else:
                    if memory.stage == "arming":
                        memory.start_from_result(result, frame_width, now)
                    divider_ok = memory.update_divider(result, frame_width, now)
                    if divider_ok:
                        memory.update_left_outer(result)
                        memory_mode = True
                        guide = memory.guide()
                        guide_error_for_finish = None
                        if guide is not None:
                            look_x = guide["far_x"] + (
                                guide["near_x"] - guide["far_x"]) * \
                                base.LANE_CHANGE_LOOKAHEAD_RATIO
                            guide_error_for_finish = base.normalized_error(
                                look_x, frame_width)
                        reached_left = (
                            memory.crossed_to_right_of_camera(frame_width) and
                            memory.left_seen_frames >= LEFT_ACQUIRE_FRAMES and
                            guide_error_for_finish is not None and
                            abs(guide_error_for_finish) <= 0.20)
                        if reached_left:
                            _finish_lane_change(self)
                            current_lane = "left"
                            target_lane = "left"
                            lane_change_active = False
                            memory_mode = True
                    else:
                        memory.note = "divider missing beyond hold interval"

            # After crossing, keep a y-dependent LEFT/MIDDLE memory.  A raw
            # label is accepted only when it matches the saved line geometry
            # at far, middle and near y locations.  If a later raw pair moves
            # away from those remembered lines, immediately fall back to this
            # memory instead of steering from the swapped pair.
            if (not lane_change_active and current_lane == "left" and
                    target_lane == "left" and memory.stage in
                    ("acquire-left", "left-hold", "normal-left")):
                divider_ok = memory.update_divider(result, frame_width, now)
                if divider_ok:
                    memory.update_left_outer(result)
                    observed_pair = None
                    observed = result.get("observed_boundaries", {})
                    if ("left" in observed and "middle" in observed):
                        observed_pair = (observed["left"], observed["middle"])
                    if _pair_matches_y_memory(observed_pair):
                        memory.normal_pair_frames = min(
                            NORMAL_LEFT_PAIR_FRAMES,
                            memory.normal_pair_frames + 1)
                    else:
                        memory.normal_pair_frames = 0
                        if memory.stage == "normal-left":
                            memory.stage = "left-hold"
                            memory.note = (
                                "detected pair disagreed with x(y) memory; "
                                "holding remembered left lane")
                    if (memory.stage != "normal-left" and
                            memory.normal_pair_frames >=
                            NORMAL_LEFT_PAIR_FRAMES):
                        memory.stage = "normal-left"
                        memory.note = (
                            "left/middle matched saved x(y) for {0} frames"
                            .format(NORMAL_LEFT_PAIR_FRAMES))
                    memory_mode = memory.stage != "normal-left"
                else:
                    if memory.stage != "normal-left":
                        memory.note = (
                            "left pair not acquired and divider expired")
                    memory_mode = False

            direct_observed = result.get(
                "observed_boundaries", result.get("boundaries", {}))
            observed_names = sorted(direct_observed.keys())

            base_pair = base.lane_boundaries(result, target_lane)
            if memory_mode:
                virtual = memory.virtual_boundaries()
                pair = None if virtual is None else (virtual["left"],
                                                       virtual["middle"])
                guide = memory.guide()
                drive_detection_ready = memory.valid(now) and guide is not None
            else:
                pair = base_pair
                guide = base.transition_path(result, current_lane, target_lane)
                needed = (("left", "middle") if current_lane == "left"
                          else ("middle", "right"))
                drive_detection_ready = all(
                    name in result.get("boundaries", {}) for name in needed)

            if drive_enabled and drive_detection_ready:
                self.drive_missing_frames = 0
            elif drive_enabled:
                self.drive_missing_frames += 1
            else:
                self.drive_missing_frames = 0

            space_error = None
            if pair is not None:
                target_x = (pair[0]["near_x"] + pair[1]["near_x"]) * 0.5
                space_error = base.normalized_error(target_x, frame_width)

            guide_error = None
            if guide is not None:
                lookahead = (base.LANE_CHANGE_LOOKAHEAD_RATIO
                             if lane_change_active else
                             base.LANE_KEEP_LOOKAHEAD_RATIO)
                look_x = guide["far_x"] + (
                    guide["near_x"] - guide["far_x"]) * lookahead
                guide_error = base.normalized_error(look_x, frame_width)

            steer_deg = 0.0
            speed_cmd = 0
            if (steering_enabled and base.steering_hardware is not None and
                    guide_error is not None):
                self.steering_stable_frames += 1
                if lane_change_active:
                    kp = base.KP_DEG
                    kd = base.KD_DEG
                    min_steer = base.MIN_ACTIVE_STEER_DEG
                    max_steer = base.MAX_TEST_STEER_DEG
                    deadband = 0.0
                else:
                    kp = base.LANE_KEEP_KP_DEG
                    kd = base.LANE_KEEP_KD_DEG
                    min_steer = base.LANE_KEEP_MIN_STEER_DEG
                    max_steer = base.LANE_KEEP_MAX_STEER_DEG
                    deadband = base.LANE_KEEP_DEADBAND
                derivative = guide_error - self.previous_guide_error
                self.previous_guide_error = guide_error
                if abs(guide_error) >= deadband:
                    steer_deg = steer_sign * (kp * guide_error + kd * derivative)
                    steer_deg = max(-max_steer, min(max_steer, steer_deg))
                    if abs(steer_deg) < min_steer:
                        steer_deg = (min_steer if steer_deg > 0.0
                                     else -min_steer)
                if self.steering_stable_frames >= base.STEERING_STABLE_FRAMES:
                    with base.control_lock:
                        with base.state_lock:
                            live_drive = base.state["drive_enabled"]
                            live_steering = base.state["steering_enabled"]
                        if (live_drive and self.drive_missing_frames <=
                                base.DRIVE_LOST_FRAMES_TO_STOP):
                            base.steering_hardware.drive(configured_speed,
                                                         steer_deg)
                            speed_cmd = configured_speed
                        elif live_steering and not live_drive:
                            base.steering_hardware.set_steering(steer_deg)
                        else:
                            base.steering_hardware.stop()
                else:
                    base.steering_hardware.stop()
            else:
                self.steering_stable_frames = 0
                self.previous_guide_error = 0.0
                if base.steering_hardware is not None:
                    base.steering_hardware.stop()

            if (drive_enabled and self.drive_missing_frames >
                    base.DRIVE_LOST_FRAMES_TO_STOP):
                _stop_for_loss("divider or lane boundaries lost")
                drive_enabled = False
                steer_deg = 0.0
                speed_cmd = 0
                control_error = "divider or lane boundaries lost; motor stopped"

            if publish_frame:
                preview_result = (_memory_preview_result(result)
                                  if memory_mode else result)
                preview = base.draw_preview(
                    frame, preview_result, target_lane, current_lane,
                    lookahead_ratio=(base.LANE_CHANGE_LOOKAHEAD_RATIO
                                     if lane_change_active else
                                     base.LANE_KEEP_LOOKAHEAD_RATIO))
                if memory_mode:
                    preview = _draw_memory_overlay(preview, frame_width)
                encode_ok, encoded = cv2.imencode(
                    ".jpg", preview,
                    [int(cv2.IMWRITE_JPEG_QUALITY), base.JPEG_QUALITY])
                if not encode_ok:
                    raise RuntimeError("JPEG encoding failed")

            with base.state_lock:
                base.state["camera_ok"] = True
                base.state["frames_received"] += 1
                base.state["last_frame_at"] = now
                base.state["last_error"] = control_error
                base.state["observed_boundaries"] = observed_names
                base.state["estimated_boundaries"] = (
                    ["left(memory)"] if memory_mode and
                    memory.left_outer is None else [])
                base.state["target_available"] = pair is not None
                base.state["space_error"] = space_error
                base.state["guide_error"] = guide_error
                base.state["raw_line_count"] = len(result.get("models", []))
                base.state["cluster_count"] = len(result.get("clusters", []))
                base.state["detection_mode"] = result.get(
                    "detection_mode", self.detector.detection_mode)
                base.state["steer_deg"] = steer_deg
                base.state["speed_cmd"] = speed_cmd
                base.state["divider_memory_stage"] = memory.stage
                base.state["divider_memory_valid"] = memory.valid(now)
                base.state["divider_memory_age_frames"] = memory.age_frames
                base.state["divider_memory_left_frames"] = memory.left_seen_frames
                base.state["divider_memory_y_match_frames"] = (
                    memory.normal_pair_frames)
                base.state["divider_memory_y_match_error_px"] = (
                    memory.y_match_error_px)
                base.state["divider_memory_note"] = memory.note
                base.state["divider_memory_width"] = memory.width_near
                base.state["start_divider_ready"] = (
                    memory.start_reference_frames >= START_REFERENCE_FRAMES)
                base.state["start_divider_locked"] = (
                    memory.start_reference_locked)
                base.state["start_divider_frames"] = (
                    memory.start_reference_frames)
                base.state["start_divider_note"] = memory.start_reference_note

            if publish_frame:
                with self.condition:
                    self.latest_jpeg = encoded.tobytes()
                    self.sequence += 1
                    self.condition.notify_all()

        except Exception as exc:
            with base.control_lock:
                with base.state_lock:
                    base.state["last_error"] = str(exc)
                    base.state["steering_enabled"] = False
                    base.state["drive_enabled"] = False
                    base.state["lane_change_active"] = False
                    base.state["lane_change_direction"] = None
                    base.state["lane_change_started_at"] = None
                    base.state["lane_change_settle_until"] = None
                    base.state["steer_deg"] = 0.0
                    base.state["speed_cmd"] = 0
                if base.steering_hardware is not None:
                    base.steering_hardware.stop()
            memory.reset()
            time.sleep(0.02)

    with base.state_lock:
        base.state["camera_ok"] = False


def _api_target_with_divider_memory():
    lane = base.requested_lane()
    if lane is None:
        return jsonify({"ok": False, "message": "lane must be left or right"}), 400
    with base.state_lock:
        if base.state["drive_enabled"]:
            if (base.state["current_lane"] == "right" and lane == "left"):
                started_at = time.time()
                base.state["target_lane"] = "left"
                base.state["lane_change_active"] = True
                base.state["lane_change_direction"] = "right_to_left_memory"
                base.state["lane_change_started_at"] = started_at
                base.state["last_stop_reason"] = None
                memory.arm(started_at)
                return jsonify({
                    "ok": True,
                    "target_lane": "left",
                    "lane_change_active": True,
                    "mode": "remembered_center_divider",
                })
            if lane == base.state["current_lane"]:
                return jsonify({"ok": True, "target_lane": lane})
            return jsonify({
                "ok": False,
                "message": "this version supports right-to-left only",
            }), 409
        base.state["target_lane"] = lane
    return jsonify({"ok": True, "target_lane": lane})


_original_current = base.app.view_functions["api_current"]
_original_stop = base.app.view_functions["api_stop"]
_original_drive = base.app.view_functions["api_drive"]


def _api_current_and_reset_memory():
    result = _original_current()
    with base.state_lock:
        if not base.state["drive_enabled"]:
            memory.reset()
            memory.clear_start_reference()
    return result


def _api_stop_and_reset_memory():
    result = _original_stop()
    memory.reset()
    memory.clear_start_reference()
    return result


def _api_drive_and_lock_start_divider():
    """Require the pre-drive divider snapshot before moving the motor."""
    payload = base.request.get_json(force=True, silent=True) or {}
    enabled = payload.get("enabled") is True
    if enabled:
        with base.state_lock:
            starting_right = (base.state["current_lane"] == "right" and
                              base.state["target_lane"] == "right")
        if starting_right and not memory.lock_start_reference():
            return jsonify({
                "ok": False,
                "message": (
                    "Wait until the start divider is stable "
                    "(MIDDLE/RIGHT for 6 frames)."),
            }), 409
    result = _original_drive()
    if not enabled:
        memory.clear_start_reference()
    return result


def _install_memory_dashboard():
    base.state.update({
        "divider_memory_stage": "idle",
        "divider_memory_valid": False,
        "divider_memory_age_frames": 0,
        "divider_memory_left_frames": 0,
        "divider_memory_y_match_frames": 0,
        "divider_memory_y_match_error_px": None,
        "divider_memory_note": "idle",
        "divider_memory_width": None,
        "start_divider_ready": False,
        "start_divider_locked": False,
        "start_divider_frames": 0,
        "start_divider_note": "waiting for middle/right",
    })
    base.camera._process_loop = types.MethodType(
        _process_loop_with_divider_memory, base.camera)
    base.app.view_functions["api_target"] = _api_target_with_divider_memory
    base.app.view_functions["api_current"] = _api_current_and_reset_memory
    base.app.view_functions["api_stop"] = _api_stop_and_reset_memory
    base.app.view_functions["api_drive"] = _api_drive_and_lock_start_divider


def main(argv=None):
    _install_memory_dashboard()
    print("================================================")
    print("Centre-divider memory dashboard (separate version)")
    print("Right -> left: save middle/right, cross divider, then acquire left/middle")
    print("================================================")
    base.main(argv if argv is not None else sys.argv[1:])


if __name__ == "__main__":
    main()
