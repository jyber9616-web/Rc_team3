#!/usr/bin/env python3
"""Browser dashboard for red-tape lane preview and guarded AutoCar driving."""
from __future__ import print_function

import argparse
import threading
import time

import cv2
from flask import Flask, Response, jsonify, request

from opencv_lane_space_preview import (
    CAM_FPS,
    CAM_HEIGHT,
    CAM_WIDTH,
    MultiLaneDetector,
    complete_boundaries,
    draw_preview,
    lane_boundaries,
    transition_path,
)


HOST = "0.0.0.0"
PORT = 8002
JPEG_QUALITY = 68
STREAM_FPS = 30
LANE_CHANGE_LOOKAHEAD_RATIO = 0.70
LANE_KEEP_LOOKAHEAD_RATIO = 0.45
HARDWARE_MAX_STEER_DEG = 30.0
KP_DEG = 70.0
KD_DEG = 10.0
MAX_TEST_STEER_DEG = 30.0
MIN_ACTIVE_STEER_DEG = 18.0
LANE_KEEP_KP_DEG = 45.0
LANE_KEEP_KD_DEG = 6.0
LANE_KEEP_MIN_STEER_DEG = 8.0
LANE_KEEP_MAX_STEER_DEG = 24.0
LANE_KEEP_DEADBAND = 0.025
STEERING_STABLE_FRAMES = 1
MIN_RUNNING_SPEED = 18
DRIVE_SPEED = 47
DRIVE_LOST_FRAMES_TO_STOP = 15
LANE_CHANGE_TIMEOUT_SEC = 12.0
LANE_CHANGE_SETTLE_SEC = 1.5
LANE_CHANGE_CROSS_MARGIN_RATIO = 0.07
LANE_CHANGE_CENTER_TOLERANCE = 0.15
LANE_CHANGE_CENTERED_FRAMES = 4

app = Flask(__name__)
state_lock = threading.RLock()
control_lock = threading.RLock()
stop_event = threading.Event()

state = {
    "current_lane": "right",
    "target_lane": "right",
    "camera_ok": False,
    "frames_received": 0,
    "last_frame_at": None,
    "last_error": None,
    "observed_boundaries": [],
    "estimated_boundaries": [],
    "target_available": False,
    "space_error": None,
    "guide_error": None,
    "raw_line_count": 0,
    "cluster_count": 0,
    "detection_mode": "dark",
    "steering_available": False,
    "steering_enabled": False,
    "steer_deg": 0.0,
    "steer_sign": 1.0,
    "drive_available": False,
    "drive_enabled": False,
    "speed_cmd": 0,
    "configured_speed": DRIVE_SPEED,
    "lane_change_active": False,
    "lane_change_direction": None,
    "lane_change_started_at": None,
    "lane_change_settle_until": None,
    "last_stop_reason": None,
}


class AutoCarHardware(object):
    def __init__(self):
        from pop import Pilot
        self.car = Pilot.AutoCar()
        self.lock = threading.RLock()
        self.stop()

    def set_steering(self, steer_deg):
        steer_deg = max(-MAX_TEST_STEER_DEG,
                        min(MAX_TEST_STEER_DEG, float(steer_deg)))
        with self.lock:
            # Safety invariant: never issue forward/backward in this file.
            self.car.stop()
            self.car.steering = steer_deg / HARDWARE_MAX_STEER_DEG

    def drive(self, speed, steer_deg):
        speed = max(MIN_RUNNING_SPEED, int(abs(speed)))
        steer_deg = max(-MAX_TEST_STEER_DEG,
                        min(MAX_TEST_STEER_DEG, float(steer_deg)))
        with self.lock:
            self.car.steering = steer_deg / HARDWARE_MAX_STEER_DEG
            try:
                self.car.forward(speed)
            except TypeError:
                self.car.forward()

    def stop(self):
        with self.lock:
            self.car.stop()
            self.car.steering = 0.0


steering_hardware = None


def normalized_error(x_value, frame_width):
    if x_value is None:
        return None
    return (float(x_value) - frame_width * 0.5) / (frame_width * 0.5)


class LaneCameraService(object):
    def __init__(self):
        self.cap = None
        self.thread = None
        self.process_thread = None
        self.raw_condition = threading.Condition()
        self.latest_frame = None
        self.raw_sequence = 0
        self.condition = threading.Condition()
        self.latest_jpeg = None
        self.sequence = 0
        self.detector = MultiLaneDetector(current_lane=state["current_lane"])
        self.previous_guide_error = 0.0
        self.steering_stable_frames = 0
        self.drive_missing_frames = 0
        self.active_lane_change_started_at = None
        self.lane_change_centered_frames = 0

    def open(self):
        try:
            from pop import Util
            pipeline = Util.gstrmer(
                width=CAM_WIDTH,
                height=CAM_HEIGHT,
                fps=CAM_FPS,
                flip=0,
            )
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if not self.cap.isOpened():
                self.cap.release()
                self.cap = None
                raise RuntimeError("GStreamer camera could not be opened")
            return True
        except Exception as exc:
            with state_lock:
                state["camera_ok"] = False
                state["last_error"] = str(exc)
            print("[ERROR] camera:", exc)
            return False

    def start(self):
        if self.cap is None and not self.open():
            return False
        if self.thread is not None and self.thread.is_alive():
            return True
        self.thread = threading.Thread(target=self._capture_loop)
        self.thread.daemon = True
        self.thread.start()
        self.process_thread = threading.Thread(target=self._process_loop)
        self.process_thread.daemon = True
        self.process_thread.start()
        return True

    def _capture_loop(self):
        failures = 0
        while not stop_event.is_set():
            ok, frame = self.cap.read()
            if not ok or frame is None:
                failures += 1
                with control_lock:
                    with state_lock:
                        state["camera_ok"] = failures < 30
                        state["last_error"] = "camera frame read failed"
                        state["steering_enabled"] = False
                        state["drive_enabled"] = False
                        state["lane_change_active"] = False
                        state["lane_change_direction"] = None
                        state["lane_change_started_at"] = None
                        state["lane_change_settle_until"] = None
                        state["steer_deg"] = 0.0
                        state["speed_cmd"] = 0
                    if steering_hardware is not None:
                        steering_hardware.stop()
                time.sleep(0.05)
                continue

            failures = 0
            with self.raw_condition:
                # Replace the previous frame instead of queueing it. The
                # processor therefore never steers from an old camera frame.
                self.latest_frame = frame
                self.raw_sequence += 1
                self.raw_condition.notify_all()

    def _process_loop(self):
        captured_frames = 0
        last_raw_sequence = 0
        publish_every = max(1, int(round(float(CAM_FPS) / STREAM_FPS)))
        while not stop_event.is_set():
            with self.raw_condition:
                self.raw_condition.wait_for(
                    lambda: (self.raw_sequence != last_raw_sequence or
                             stop_event.is_set()),
                    timeout=1.0,
                )
                if stop_event.is_set():
                    break
                if self.latest_frame is None:
                    continue
                frame = self.latest_frame
                last_raw_sequence = self.raw_sequence

            captured_frames += 1
            # Steering is calculated for every camera frame. STREAM_FPS can be
            # lowered independently if a particular Wi-Fi link is unstable.
            publish_frame = captured_frames % publish_every == 0
            with state_lock:
                current_lane = state["current_lane"]
                target_lane = state["target_lane"]
                steering_enabled = state["steering_enabled"]
                steer_sign = state["steer_sign"]
                drive_enabled = state["drive_enabled"]
                configured_speed = state["configured_speed"]
                lane_change_active = state["lane_change_active"]
                lane_change_started_at = state["lane_change_started_at"]
                lane_change_settle_until = state["lane_change_settle_until"]

            # A lane change moves the middle divider across the image much
            # faster than normal lane keeping. Do not freeze that movement.
            self.detector.allow_fast_motion = lane_change_active

            if lane_change_active:
                if lane_change_started_at != self.active_lane_change_started_at:
                    self.active_lane_change_started_at = lane_change_started_at
                    self.lane_change_centered_frames = 0
            else:
                self.active_lane_change_started_at = None
                self.lane_change_centered_frames = 0

            if self.detector.current_lane != current_lane:
                self.detector.current_lane = current_lane
                self.detector.tracks = {}
                self.detector.missing = {"left": 0, "middle": 0, "right": 0}

            try:
                result = self.detector.detect(frame)
                control_error = None
                if lane_change_active:
                    elapsed = time.time() - lane_change_started_at
                    if elapsed > LANE_CHANGE_TIMEOUT_SEC:
                        with control_lock:
                            with state_lock:
                                state["drive_enabled"] = False
                                state["steering_enabled"] = False
                                state["lane_change_active"] = False
                                state["lane_change_direction"] = None
                                state["lane_change_started_at"] = None
                                state["lane_change_settle_until"] = None
                                state["speed_cmd"] = 0
                                state["last_stop_reason"] = "lane change timeout"
                            steering_hardware.stop()
                        drive_enabled = False
                        steering_enabled = False
                        lane_change_active = False
                        control_error = "lane change timeout; motor stopped"
                    elif current_lane == "right" and target_lane == "left":
                        middle = result["boundaries"].get("middle")
                        target_pair = lane_boundaries(result, "left")
                        cross_x = frame.shape[1] * (
                            0.5 + LANE_CHANGE_CROSS_MARGIN_RATIO)
                        target_center_error = None
                        if target_pair is not None:
                            target_center_x = (
                                target_pair[0]["near_x"] +
                                target_pair[1]["near_x"]) * 0.5
                            target_center_error = normalized_error(
                                target_center_x, frame.shape[1])
                        centered_in_left_lane = (
                            middle is not None and
                            middle["near_x"] > cross_x and
                            target_center_error is not None and
                            abs(target_center_error) <=
                            LANE_CHANGE_CENTER_TOLERANCE)
                        if centered_in_left_lane:
                            self.lane_change_centered_frames += 1
                        else:
                            self.lane_change_centered_frames = 0
                        if (self.lane_change_centered_frames >=
                                LANE_CHANGE_CENTERED_FRAMES):
                            current_lane = "left"
                            lane_change_active = False
                            self.detector.current_lane = "left"
                            self.detector.allow_fast_motion = False
                            self.drive_missing_frames = 0
                            self.previous_guide_error = 0.0
                            with state_lock:
                                if state["lane_change_active"]:
                                    state["current_lane"] = "left"
                                    state["lane_change_active"] = False
                                    state["lane_change_direction"] = None
                                    state["lane_change_started_at"] = None
                                    state["lane_change_settle_until"] = (
                                        time.time() + LANE_CHANGE_SETTLE_SEC)
                            lane_change_settle_until = (
                                time.time() + LANE_CHANGE_SETTLE_SEC)

                overlay = None
                lookahead_ratio = (LANE_CHANGE_LOOKAHEAD_RATIO
                                   if lane_change_active
                                   else LANE_KEEP_LOOKAHEAD_RATIO)
                if publish_frame:
                    overlay = draw_preview(
                        frame, result, target_lane, current_lane,
                        lookahead_ratio=lookahead_ratio)
                direct_observed = result.get(
                    "observed_boundaries", result["boundaries"])
                observed = sorted(direct_observed.keys())
                completed, estimated = complete_boundaries(result)
                pair = lane_boundaries(result, target_lane)
                required_names = (("left", "middle") if current_lane == "left"
                                  else ("middle", "right"))
                settling_after_change = (
                    lane_change_settle_until is not None and
                    time.time() < lane_change_settle_until)
                if (lane_change_settle_until is not None and
                        not settling_after_change):
                    lane_change_settle_until = None
                    with state_lock:
                        state["lane_change_settle_until"] = None
                if lane_change_active:
                    # MIDDLE is dashed, so retain its tracked position across
                    # the physical gaps instead of cancelling immediately.
                    drive_detection_ready = "middle" in result["boundaries"]
                elif settling_after_change:
                    # The new LEFT outer line enters the camera after MIDDLE
                    # is crossed. Continue briefly using the completed pair.
                    drive_detection_ready = pair is not None
                else:
                    # Keep using the short-lived tracked line through a dash
                    # or one blurred camera frame.  The detector removes a
                    # stale track itself, so this cannot keep driving forever.
                    drive_detection_ready = all(
                        name in result["boundaries"] for name in required_names)
                if drive_enabled and drive_detection_ready:
                    self.drive_missing_frames = 0
                elif drive_enabled:
                    self.drive_missing_frames += 1
                else:
                    self.drive_missing_frames = 0

                space_error = None
                if pair is not None:
                    first, second = pair
                    target_x = (first["near_x"] + second["near_x"]) * 0.5
                    space_error = normalized_error(target_x, frame.shape[1])

                guide_error = None
                guide = transition_path(result, current_lane, target_lane)
                if guide is not None:
                    look_x = guide["far_x"] + (
                        guide["near_x"] - guide["far_x"]) * lookahead_ratio
                    guide_error = normalized_error(look_x, frame.shape[1])

                steer_deg = 0.0
                if (steering_enabled and steering_hardware is not None and
                        guide_error is not None):
                    self.steering_stable_frames += 1
                    if lane_change_active:
                        kp = KP_DEG
                        kd = KD_DEG
                        min_steer = MIN_ACTIVE_STEER_DEG
                        deadband = 0.0
                    else:
                        # After the divider is crossed, use gentle centring
                        # inside the new lane instead of continuing the hard
                        # lane-change turn.
                        kp = LANE_KEEP_KP_DEG
                        kd = LANE_KEEP_KD_DEG
                        min_steer = LANE_KEEP_MIN_STEER_DEG
                        max_steer = LANE_KEEP_MAX_STEER_DEG
                        deadband = LANE_KEEP_DEADBAND
                    if lane_change_active:
                        max_steer = MAX_TEST_STEER_DEG
                    derivative = guide_error - self.previous_guide_error
                    self.previous_guide_error = guide_error
                    if abs(guide_error) < deadband:
                        steer_deg = 0.0
                    else:
                        steer_deg = steer_sign * (
                            kp * guide_error + kd * derivative)
                        steer_deg = max(-max_steer,
                                        min(max_steer, steer_deg))
                        if abs(steer_deg) < min_steer:
                            steer_deg = (min_steer if steer_deg > 0.0
                                         else -min_steer)
                    if self.steering_stable_frames >= STEERING_STABLE_FRAMES:
                        with control_lock:
                            with state_lock:
                                live_drive = state["drive_enabled"]
                                live_steering = state["steering_enabled"]
                                if (live_drive and self.drive_missing_frames <=
                                        DRIVE_LOST_FRAMES_TO_STOP):
                                    steering_hardware.drive(
                                        configured_speed, steer_deg)
                                    speed_cmd = configured_speed
                                elif live_steering and not live_drive:
                                    steering_hardware.set_steering(steer_deg)
                                    speed_cmd = 0
                                else:
                                    steering_hardware.stop()
                                    speed_cmd = 0
                    else:
                        steering_hardware.stop()
                        steer_deg = 0.0
                        speed_cmd = 0
                else:
                    self.steering_stable_frames = 0
                    self.previous_guide_error = 0.0
                    if steering_hardware is not None:
                        steering_hardware.stop()
                    speed_cmd = 0

                if (drive_enabled and self.drive_missing_frames >
                        DRIVE_LOST_FRAMES_TO_STOP):
                    with control_lock:
                        with state_lock:
                            state["drive_enabled"] = False
                            state["steering_enabled"] = False
                            state["lane_change_active"] = False
                            state["lane_change_direction"] = None
                            state["lane_change_started_at"] = None
                            state["lane_change_settle_until"] = None
                            state["speed_cmd"] = 0
                            state["last_stop_reason"] = "lane boundaries lost"
                        steering_hardware.stop()
                    drive_enabled = False
                    steer_deg = 0.0
                    speed_cmd = 0

                encoded = None
                if publish_frame:
                    encode_ok, encoded = cv2.imencode(
                        ".jpg", overlay,
                        [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
                    if not encode_ok:
                        raise RuntimeError("JPEG encoding failed")

                with state_lock:
                    state["camera_ok"] = True
                    state["frames_received"] += 1
                    state["last_frame_at"] = time.time()
                    state["last_error"] = control_error
                    state["observed_boundaries"] = observed
                    state["estimated_boundaries"] = sorted(estimated)
                    state["target_available"] = pair is not None
                    state["space_error"] = space_error
                    state["guide_error"] = guide_error
                    state["raw_line_count"] = len(result["models"])
                    state["cluster_count"] = len(result["clusters"])
                    # Stay compatible if an older lane-space module is still
                    # present on the RC car and omits this optional field.
                    state["detection_mode"] = result.get(
                        "detection_mode", self.detector.detection_mode)
                    state["steer_deg"] = steer_deg
                    state["speed_cmd"] = speed_cmd

                if publish_frame:
                    with self.condition:
                        self.latest_jpeg = encoded.tobytes()
                        self.sequence += 1
                        self.condition.notify_all()

            except Exception as exc:
                with control_lock:
                    with state_lock:
                        state["last_error"] = str(exc)
                        state["steering_enabled"] = False
                        state["drive_enabled"] = False
                        state["lane_change_active"] = False
                        state["lane_change_direction"] = None
                        state["lane_change_started_at"] = None
                        state["lane_change_settle_until"] = None
                        state["steer_deg"] = 0.0
                        state["speed_cmd"] = 0
                    if steering_hardware is not None:
                        steering_hardware.stop()
                time.sleep(0.02)

        with state_lock:
            state["camera_ok"] = False

    def generate_mjpeg(self):
        last_sequence = -1
        while not stop_event.is_set():
            with self.condition:
                self.condition.wait_for(
                    lambda: self.sequence != last_sequence or stop_event.is_set(),
                    timeout=1.0,
                )
                if stop_event.is_set():
                    break
                if self.latest_jpeg is None:
                    continue
                jpeg = self.latest_jpeg
                last_sequence = self.sequence
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
            )

    def close(self):
        stop_event.set()
        with self.condition:
            self.condition.notify_all()
        with self.raw_condition:
            self.raw_condition.notify_all()
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        if steering_hardware is not None:
            steering_hardware.stop()


camera = LaneCameraService()


@app.route("/")
def index():
    return """
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>RC카 차선 인식 미리보기</title>
<style>
body { margin:0; font-family:Arial,sans-serif; background:#101217; color:#f1f1f1; }
.wrap { max-width:960px; margin:auto; padding:18px; }
.card { background:#1b1f27; border:1px solid #394150; border-radius:12px; padding:16px; margin-bottom:14px; }
h1 { margin:0 0 8px; font-size:24px; }
.safe { color:#74e69a; font-weight:bold; }
.video { display:block; width:100%; max-width:640px; image-rendering:auto; background:#000; border-radius:10px; border:2px solid #4b5568; }
.row { display:flex; flex-wrap:wrap; gap:8px; margin:10px 0; }
button { border:0; border-radius:9px; min-width:135px; height:44px; font-size:15px; cursor:pointer; font-weight:bold; }
.left { background:#4aa3ff; }
.right { background:#ff765f; }
.current { background:#f0c94c; }
.stop { background:#e33; color:white; }
.steer { background:#8b5cf6; color:white; }
.drive { background:#20b96b; color:white; }
pre { background:#090b0f; color:#85f29b; padding:12px; border-radius:8px; min-height:170px; overflow:auto; }
.selected { outline:3px solid white; }
</style>
</head>
<body>
<div class="wrap">
  <div class="card">
    <h1>RC카 차선 인식 WebView</h1>
    <div class="safe">시작 시 모터 정지 · 주행 시작 버튼을 눌러야 출발</div>
  </div>
  <div class="card">
    <img class="video" src="/video_feed" alt="camera stream">
  </div>
  <div class="card">
    <b>현재 RC카가 놓인 차로</b>
    <div class="row">
      <button id="current-left" class="current" onclick="setCurrent('left')">현재: 왼쪽</button>
      <button id="current-right" class="current" onclick="setCurrent('right')">현재: 오른쪽</button>
    </div>
    <b>이동 목표 차로</b>
    <div class="row">
      <button id="target-left" class="left" onclick="setTarget('left')">목표: 왼쪽</button>
      <button id="target-right" class="right" onclick="setTarget('right')">목표: 오른쪽</button>
      <button class="stop" onclick="safeReset()">목표 취소</button>
    </div>
    <b>조향 단독 안전 시험</b>
    <div class="row">
      <button class="steer" onclick="setSteering(true)">조향 시작</button>
      <button class="stop" onclick="setSteering(false)">조향 정지</button>
    </div>
    <small>조향 단독 시험은 모터 OFF입니다.</small>
    <b>저속 자동 주행</b>
    <div class="row">
      <button class="drive" onclick="setDrive(true)">주행 시작</button>
      <button class="stop" onclick="setDrive(false)">긴급 정지</button>
    </div>
  </div>
  <div class="card"><pre id="status">연결 중...</pre></div>
</div>
<script>
async function postJson(url, data) {
  const response = await fetch(url, {
    method:'POST', headers:{'Content-Type':'application/json'},
    body:JSON.stringify(data || {})
  });
  return await response.json();
}
async function setCurrent(lane) {
  const result = await postJson('/api/current', {lane:lane});
  if (!result.ok) alert(result.message || '현재 차로를 변경할 수 없습니다.');
  await refresh();
}
async function setTarget(lane) {
  const result = await postJson('/api/target', {lane:lane});
  if (!result.ok) alert(result.message || '목표 차로를 변경할 수 없습니다.');
  await refresh();
}
async function safeReset() { await postJson('/api/stop', {}); await refresh(); }
async function setSteering(enabled) {
  const result = await postJson('/api/steering', {enabled:enabled});
  if (!result.ok) alert(result.message || '조향을 시작할 수 없습니다.');
  await refresh();
}
async function setDrive(enabled) {
  const result = await postJson('/api/drive', {enabled:enabled});
  if (!result.ok) alert(result.message || '주행을 시작할 수 없습니다.');
  await refresh();
}
function mark(id, selected) { document.getElementById(id).classList.toggle('selected', selected); }
async function refresh() {
  try {
    const s = await fetch('/api/status').then(r => r.json());
    document.getElementById('status').textContent = JSON.stringify(s, null, 2);
    mark('current-left', s.current_lane === 'left');
    mark('current-right', s.current_lane === 'right');
    mark('target-left', s.target_lane === 'left');
    mark('target-right', s.target_lane === 'right');
  } catch (error) {
    document.getElementById('status').textContent = '연결 오류: ' + error;
  }
}
refresh(); setInterval(refresh, 500);
</script>
</body>
</html>
"""


@app.route("/video_feed")
def video_feed():
    response = Response(
        camera.generate_mjpeg(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )
    response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate"
    response.headers["Pragma"] = "no-cache"
    return response


@app.route("/api/status")
def api_status():
    with state_lock:
        data = dict(state)
    if data["last_frame_at"] is not None:
        data["last_frame_age_sec"] = round(
            time.time() - data["last_frame_at"], 3)
    else:
        data["last_frame_age_sec"] = None
    if data["space_error"] is not None:
        data["space_error"] = round(data["space_error"], 4)
    if data["guide_error"] is not None:
        data["guide_error"] = round(data["guide_error"], 4)
    data["motor_enabled"] = bool(data["drive_enabled"])
    return jsonify(data)


def requested_lane():
    payload = request.get_json(force=True, silent=True) or {}
    lane = payload.get("lane")
    if lane not in ("left", "right"):
        return None
    return lane


@app.route("/api/current", methods=["POST"])
def api_current():
    lane = requested_lane()
    if lane is None:
        return jsonify({"ok": False, "message": "lane must be left or right"}), 400
    with state_lock:
        if state["drive_enabled"]:
            return jsonify({"ok": False, "message": "긴급 정지 후 현재 차로를 변경하세요."}), 409
        state["current_lane"] = lane
        state["target_lane"] = lane
        state["lane_change_active"] = False
        state["lane_change_direction"] = None
        state["lane_change_started_at"] = None
        state["lane_change_settle_until"] = None
    return jsonify({"ok": True, "current_lane": lane, "target_lane": lane})


@app.route("/api/target", methods=["POST"])
def api_target():
    lane = requested_lane()
    if lane is None:
        return jsonify({"ok": False, "message": "lane must be left or right"}), 400
    with state_lock:
        if state["drive_enabled"]:
            if (state["current_lane"] == "right" and lane == "left"):
                state["target_lane"] = "left"
                state["lane_change_active"] = True
                state["lane_change_direction"] = "right_to_left"
                state["lane_change_started_at"] = time.time()
                return jsonify({
                    "ok": True,
                    "target_lane": "left",
                    "lane_change_active": True,
                })
            if lane == state["current_lane"]:
                return jsonify({"ok": True, "target_lane": lane})
            return jsonify({"ok": False, "message": "현재 버전은 오른쪽에서 왼쪽 차선 변경만 지원합니다."}), 409
        state["target_lane"] = lane
    return jsonify({"ok": True, "target_lane": lane})


@app.route("/api/stop", methods=["POST"])
def api_stop():
    with control_lock:
        with state_lock:
            state["target_lane"] = state["current_lane"]
            state["steering_enabled"] = False
            state["drive_enabled"] = False
            state["lane_change_active"] = False
            state["lane_change_direction"] = None
            state["lane_change_started_at"] = None
            state["lane_change_settle_until"] = None
            state["steer_deg"] = 0.0
            state["speed_cmd"] = 0
            state["last_stop_reason"] = "user stop"
        if steering_hardware is not None:
            steering_hardware.stop()
    return jsonify({
        "ok": True,
        "message": "motor stopped, steering centred, target reset",
    })


@app.route("/api/steering", methods=["POST"])
def api_steering():
    payload = request.get_json(force=True, silent=True) or {}
    enabled = payload.get("enabled") is True
    if enabled and steering_hardware is None:
        return jsonify({
            "ok": False,
            "message": "서버를 --steering-only 옵션으로 다시 실행하세요.",
        }), 409
    if enabled:
        with state_lock:
            ready = (
                state["camera_ok"] and
                state["target_available"] and
                state["guide_error"] is not None and
                state["current_lane"] == state["target_lane"] and
                state["last_frame_at"] is not None and
                time.time() - state["last_frame_at"] < 0.5
            )
        if not ready:
            return jsonify({
                "ok": False,
                "message": "현재 차로와 목표 차로를 같게 하고 차선 2개가 검출된 상태에서 시작하세요.",
            }), 409
    with control_lock:
        with state_lock:
            state["steering_enabled"] = enabled
            if not enabled:
                state["drive_enabled"] = False
                state["target_lane"] = state["current_lane"]
                state["lane_change_active"] = False
                state["lane_change_direction"] = None
                state["lane_change_started_at"] = None
                state["lane_change_settle_until"] = None
                state["steer_deg"] = 0.0
                state["speed_cmd"] = 0
        if not enabled and steering_hardware is not None:
            steering_hardware.stop()
    return jsonify({"ok": True, "steering_enabled": enabled})


@app.route("/api/drive", methods=["POST"])
def api_drive():
    payload = request.get_json(force=True, silent=True) or {}
    enabled = payload.get("enabled") is True
    if enabled:
        with state_lock:
            required = (("left", "middle") if state["current_lane"] == "left"
                        else ("middle", "right"))
            ready = (
                state["drive_available"] and
                state["camera_ok"] and
                state["current_lane"] == state["target_lane"] and
                state["guide_error"] is not None and
                all(name in state["observed_boundaries"] for name in required) and
                state["last_frame_at"] is not None and
                time.time() - state["last_frame_at"] < 0.5
            )
        if not ready:
            return jsonify({
                "ok": False,
                "message": "--drive로 실행하고 현재/목표 차로를 같게 한 뒤 실제 경계선 2개가 검출될 때 시작하세요.",
            }), 409
    with control_lock:
        with state_lock:
            state["drive_enabled"] = enabled
            state["steering_enabled"] = enabled
            if enabled:
                state["last_stop_reason"] = None
            if not enabled:
                state["target_lane"] = state["current_lane"]
                state["lane_change_active"] = False
                state["lane_change_direction"] = None
                state["lane_change_started_at"] = None
                state["lane_change_settle_until"] = None
                state["speed_cmd"] = 0
                state["steer_deg"] = 0.0
        if not enabled and steering_hardware is not None:
            steering_hardware.stop()
    return jsonify({"ok": True, "drive_enabled": enabled})


def main(argv=None):
    global steering_hardware
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--current-lane", choices=("left", "right"), default="right")
    parser.add_argument(
        "--detection-mode", choices=("dark", "red"), default="dark")
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument(
        "--steering-only", action="store_true",
        help="enable front steering test; drive motor remains stopped")
    mode_group.add_argument(
        "--drive", action="store_true",
        help="allow guarded forward driving from the WebView")
    parser.add_argument("--speed", type=int, default=DRIVE_SPEED)
    parser.add_argument(
        "--steer-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    args = parser.parse_args(argv)
    if args.speed < 45 or args.speed > 50:
        parser.error("--speed must be between 45 and 50")
    if args.steering_only or args.drive:
        steering_hardware = AutoCarHardware()
    with state_lock:
        state["current_lane"] = args.current_lane
        state["target_lane"] = args.current_lane
        state["detection_mode"] = args.detection_mode
        state["steering_available"] = args.steering_only or args.drive
        state["steering_enabled"] = False
        state["steer_sign"] = args.steer_sign
        state["drive_available"] = args.drive
        state["drive_enabled"] = False
        state["configured_speed"] = args.speed
        state["speed_cmd"] = 0
        state["lane_change_active"] = False
        state["lane_change_direction"] = None
        state["lane_change_started_at"] = None
        state["lane_change_settle_until"] = None
    camera.detector.detection_mode = args.detection_mode
    camera.detector.tracks = {}

    camera.start()
    print("================================================")
    print("Lane WebView preview")
    print("Open: http://RC_CAR_IP:{0}".format(args.port))
    print("Motor at startup: OFF / Steering at startup: OFF")
    print("Steering-only available:", args.steering_only)
    print("Drive available:", args.drive)
    print("Configured speed:", args.speed)
    print("Detection mode:", args.detection_mode)
    print("Stop server: Ctrl+C")
    print("================================================")
    try:
        app.run(
            host=args.host,
            port=args.port,
            debug=False,
            use_reloader=False,
            threaded=True,
        )
    finally:
        camera.close()
        print("Camera released. Server stopped.")


if __name__ == "__main__":
    main()
