#!/usr/bin/env python3
"""Browser dashboard for red-tape lane preview and guarded AutoCar driving."""
from __future__ import print_function

import argparse
import json
import threading
import time

import cv2
import paho.mqtt.client as mqtt
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
PORT = 8000
JPEG_QUALITY = 68
STREAM_FPS = 15
LOOKAHEAD_RATIO = 0.20
HARDWARE_MAX_STEER_DEG = 30.0
KP_DEG = 28.0
KD_DEG = 5.0
MAX_TEST_STEER_DEG = 24.0
MIN_ACTIVE_STEER_DEG = 6.0
STEERING_STABLE_FRAMES = 5
MIN_RUNNING_SPEED = 18
DRIVE_SPEED = 47
DRIVE_LOST_FRAMES_TO_STOP = 3

# ── MQTT 설정 (kwon/MQTT_연동_가이드.md 참고) ────────────────────────
MQTT_BROKER_ADDRESS = "172.20.10.5"  # 신용이 브로커 주소, 네트워크 바뀌면 여기 수정
MQTT_PORT = 1883
CAR_ID = "A"

MQTT_STATUS_TOPIC = "rcteam3/autocar/{0}/status".format(CAR_ID)
MQTT_EVENT_TOPIC = "rcteam3/autocar/{0}/event".format(CAR_ID)
MQTT_COMMAND_TOPIC = "rcteam3/autocar/{0}/command".format(CAR_ID)
MQTT_COMMAND_ALL_TOPIC = "rcteam3/autocar/all/command"
MQTT_STATUS_PUBLISH_INTERVAL = 0.1  # 10Hz

_emergency_active = False

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


def publish_status(mode, speed, steering_angle, brake):
    payload = {
        "car_id": CAR_ID,
        "timestamp": time.time(),
        "mode": mode,
        "speed": speed,
        "steering_angle": round(float(steering_angle), 2),
        "distance_to_front": None,  # 선두 차량은 항상 None
        "brake": brake,
    }
    mqtt_client.publish(MQTT_STATUS_TOPIC, json.dumps(payload), qos=0)


def publish_event(event_type, detail=None):
    payload = {
        "car_id": CAR_ID,
        "timestamp": time.time(),
        "event_type": event_type,
        "detail": detail or {},
    }
    mqtt_client.publish(MQTT_EVENT_TOPIC, json.dumps(payload), qos=1)


def _on_mqtt_connect(client, userdata, flags, rc):
    if rc == 0:
        print("[MQTT] 브로커 연결 성공")
        client.subscribe(MQTT_COMMAND_TOPIC)
        client.subscribe(MQTT_COMMAND_ALL_TOPIC)
    else:
        print("[MQTT] 연결 실패, code={0}".format(rc))


def _on_mqtt_message(client, userdata, msg):
    global _emergency_active
    try:
        data = json.loads(msg.payload.decode("utf-8"))
    except ValueError:
        return

    command = data.get("command")
    if command == "emergency_stop":
        _emergency_active = True
        _stop_all()
        publish_event("emergency_stop", {"reason": "dashboard"})
        print("[MQTT] 긴급 정지 명령 수신!")
    elif command == "stop":
        _emergency_active = False
        _stop_all()
        print("[MQTT] 정지 명령 수신")
    elif command == "start":
        _emergency_active = False
        ok, message = _set_drive(True)
        if ok:
            print("[MQTT] 시작 명령 수신 - 주행 시작")
        else:
            print("[MQTT] 시작 명령 수신했지만 준비 안 됨: {0}".format(message))


mqtt_client = mqtt.Client()
mqtt_client.on_connect = _on_mqtt_connect
mqtt_client.on_message = _on_mqtt_message
mqtt_client.will_set(
    MQTT_STATUS_TOPIC,
    payload=json.dumps({"car_id": CAR_ID, "mode": "offline"}),
    qos=1,
    retain=True,
)
mqtt_client.connect(MQTT_BROKER_ADDRESS, MQTT_PORT, 60)
mqtt_client.loop_start()


def normalized_error(x_value, frame_width):
    if x_value is None:
        return None
    return (float(x_value) - frame_width * 0.5) / (frame_width * 0.5)


class LaneCameraService(object):
    def __init__(self):
        self.cap = None
        self.thread = None
        self.condition = threading.Condition()
        self.latest_jpeg = None
        self.sequence = 0
        self.detector = MultiLaneDetector(current_lane=state["current_lane"])
        self.previous_guide_error = 0.0
        self.steering_stable_frames = 0
        self.drive_missing_frames = 0
        self.last_mqtt_status_time = 0.0

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
        return True

    def _capture_loop(self):
        failures = 0
        captured_frames = 0
        publish_every = max(1, int(round(float(CAM_FPS) / STREAM_FPS)))
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
                        state["steer_deg"] = 0.0
                        state["speed_cmd"] = 0
                    if steering_hardware is not None:
                        steering_hardware.stop()
                time.sleep(0.05)
                continue

            failures = 0
            captured_frames += 1
            # Always drain the camera, but process and transmit fewer frames.
            # This prevents old frames accumulating while keeping CPU and Wi-Fi
            # bandwidth low enough for the SODA environment.
            if captured_frames % publish_every != 0:
                continue
            with state_lock:
                current_lane = state["current_lane"]
                target_lane = state["target_lane"]
                steering_enabled = state["steering_enabled"]
                steer_sign = state["steer_sign"]
                drive_enabled = state["drive_enabled"]
                configured_speed = state["configured_speed"]

            if self.detector.current_lane != current_lane:
                self.detector.current_lane = current_lane
                self.detector.tracks = {}
                self.detector.missing = {"left": 0, "middle": 0, "right": 0}

            try:
                result = self.detector.detect(frame)
                overlay = draw_preview(
                    frame, result, target_lane, current_lane)
                direct_observed = result.get(
                    "observed_boundaries", result["boundaries"])
                observed = sorted(direct_observed.keys())
                completed, estimated = complete_boundaries(result)
                pair = lane_boundaries(result, target_lane)
                required_names = (("left", "middle") if current_lane == "left"
                                  else ("middle", "right"))
                drive_detection_ready = all(
                    name in direct_observed for name in required_names)
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
                        guide["near_x"] - guide["far_x"]) * LOOKAHEAD_RATIO
                    guide_error = normalized_error(look_x, frame.shape[1])

                steer_deg = 0.0
                if (steering_enabled and steering_hardware is not None and
                        guide_error is not None):
                    self.steering_stable_frames += 1
                    derivative = guide_error - self.previous_guide_error
                    self.previous_guide_error = guide_error
                    steer_deg = steer_sign * (
                        KP_DEG * guide_error + KD_DEG * derivative)
                    steer_deg = max(-MAX_TEST_STEER_DEG,
                                    min(MAX_TEST_STEER_DEG, steer_deg))
                    if (abs(guide_error) >= 0.03 and
                            abs(steer_deg) < MIN_ACTIVE_STEER_DEG):
                        steer_deg = (MIN_ACTIVE_STEER_DEG if steer_deg > 0.0
                                     else -MIN_ACTIVE_STEER_DEG)
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
                            state["speed_cmd"] = 0
                        steering_hardware.stop()
                    drive_enabled = False
                    steer_deg = 0.0
                    speed_cmd = 0

                encode_ok, encoded = cv2.imencode(
                    ".jpg", overlay,
                    [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
                if not encode_ok:
                    raise RuntimeError("JPEG encoding failed")

                with state_lock:
                    state["camera_ok"] = True
                    state["frames_received"] += 1
                    state["last_frame_at"] = time.time()
                    state["last_error"] = None
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

                now_mqtt = time.time()
                if now_mqtt - self.last_mqtt_status_time >= MQTT_STATUS_PUBLISH_INTERVAL:
                    if _emergency_active:
                        mqtt_mode = "emergency_stop"
                    elif drive_enabled:
                        mqtt_mode = "driving"
                    else:
                        mqtt_mode = "stopped"
                    publish_status(mqtt_mode, speed_cmd, steer_deg, False)
                    self.last_mqtt_status_time = now_mqtt

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
<title>RCì¹´ ì°¨ì  ì¸ì ë¯¸ë¦¬ë³´ê¸°</title>
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
    <h1>RCì¹´ ì°¨ì  ì¸ì WebView</h1>
    <div class="safe">ìì ì ëª¨í° ì ì§ Â· ì£¼í ìì ë²í¼ì ëë¬ì¼ ì¶ë°</div>
  </div>
  <div class="card">
    <img class="video" src="/video_feed" alt="camera stream">
  </div>
  <div class="card">
    <b>íì¬ RCì¹´ê° ëì¸ ì°¨ë¡</b>
    <div class="row">
      <button id="current-left" class="current" onclick="setCurrent('left')">íì¬: ì¼ìª½</button>
      <button id="current-right" class="current" onclick="setCurrent('right')">íì¬: ì¤ë¥¸ìª½</button>
    </div>
    <b>ì´ë ëª©í ì°¨ë¡</b>
    <div class="row">
      <button id="target-left" class="left" onclick="setTarget('left')">ëª©í: ì¼ìª½</button>
      <button id="target-right" class="right" onclick="setTarget('right')">ëª©í: ì¤ë¥¸ìª½</button>
      <button class="stop" onclick="safeReset()">ëª©í ì·¨ì</button>
    </div>
    <b>ì¡°í¥ ë¨ë ìì  ìí</b>
    <div class="row">
      <button class="steer" onclick="setSteering(true)">ì¡°í¥ ìì</button>
      <button class="stop" onclick="setSteering(false)">ì¡°í¥ ì ì§</button>
    </div>
    <small>ì¡°í¥ ë¨ë ìíì ëª¨í° OFFìëë¤.</small>
    <b>ì ì ìë ì£¼í</b>
    <div class="row">
      <button class="drive" onclick="setDrive(true)">ì£¼í ìì</button>
      <button class="stop" onclick="setDrive(false)">ê¸´ê¸ ì ì§</button>
    </div>
  </div>
  <div class="card"><pre id="status">ì°ê²° ì¤...</pre></div>
</div>
<script>
async function postJson(url, data) {
  const response = await fetch(url, {
    method:'POST', headers:{'Content-Type':'application/json'},
    body:JSON.stringify(data || {})
  });
  return await response.json();
}
async function setCurrent(lane) { await postJson('/api/current', {lane:lane}); await refresh(); }
async function setTarget(lane) { await postJson('/api/target', {lane:lane}); await refresh(); }
async function safeReset() { await postJson('/api/stop', {}); await refresh(); }
async function setSteering(enabled) {
  const result = await postJson('/api/steering', {enabled:enabled});
  if (!result.ok) alert(result.message || 'ì¡°í¥ì ììí  ì ììµëë¤.');
  await refresh();
}
async function setDrive(enabled) {
  const result = await postJson('/api/drive', {enabled:enabled});
  if (!result.ok) alert(result.message || 'ì£¼íì ììí  ì ììµëë¤.');
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
    document.getElementById('status').textContent = 'ì°ê²° ìë¥: ' + error;
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
            return jsonify({"ok": False, "message": "ê¸´ê¸ ì ì§ í íì¬ ì°¨ë¡ë¥¼ ë³ê²½íì¸ì."}), 409
        state["current_lane"] = lane
        state["target_lane"] = lane
    return jsonify({"ok": True, "current_lane": lane, "target_lane": lane})


@app.route("/api/target", methods=["POST"])
def api_target():
    lane = requested_lane()
    if lane is None:
        return jsonify({"ok": False, "message": "lane must be left or right"}), 400
    with state_lock:
        if state["drive_enabled"]:
            return jsonify({"ok": False, "message": "íì¬ ë¨ê³ììë ê¸´ê¸ ì ì§ í ëª©íë¥¼ ë³ê²½íì¸ì."}), 409
        changed = lane != state["current_lane"]
        state["target_lane"] = lane
    if changed:
        direction = "left" if lane == "left" else "right"
        publish_event("lane_change", {"direction": direction})
    return jsonify({"ok": True, "target_lane": lane})


def _stop_all():
    """모터 정지 + 조향 중앙 + 목표차로 리셋. REST(/api/stop)와 MQTT 명령이 공유."""
    with control_lock:
        with state_lock:
            state["target_lane"] = state["current_lane"]
            state["steering_enabled"] = False
            state["drive_enabled"] = False
            state["steer_deg"] = 0.0
            state["speed_cmd"] = 0
        if steering_hardware is not None:
            steering_hardware.stop()


@app.route("/api/stop", methods=["POST"])
def api_stop():
    _stop_all()
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
            "message": "ìë²ë¥¼ --steering-only ìµìì¼ë¡ ë¤ì ì¤ííì¸ì.",
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
                "message": "íì¬ ì°¨ë¡ì ëª©í ì°¨ë¡ë¥¼ ê°ê² íê³  ì°¨ì  2ê°ê° ê²ì¶ë ìíìì ììíì¸ì.",
            }), 409
    with control_lock:
        with state_lock:
            state["steering_enabled"] = enabled
            if not enabled:
                state["drive_enabled"] = False
                state["steer_deg"] = 0.0
                state["speed_cmd"] = 0
        if not enabled and steering_hardware is not None:
            steering_hardware.stop()
    return jsonify({"ok": True, "steering_enabled": enabled})


def _drive_ready():
    """--drive로 켜졌고, 카메라/차로 인식이 안전하게 준비됐는지 확인."""
    with state_lock:
        required = (("left", "middle") if state["current_lane"] == "left"
                    else ("middle", "right"))
        return (
            state["drive_available"] and
            state["camera_ok"] and
            state["current_lane"] == state["target_lane"] and
            state["guide_error"] is not None and
            all(name in state["observed_boundaries"] for name in required) and
            state["last_frame_at"] is not None and
            time.time() - state["last_frame_at"] < 0.5
        )


def _set_drive(enabled):
    """주행 on/off. REST(/api/drive)와 MQTT 명령이 공유. 반환: (성공 여부, 실패 메시지)."""
    if enabled and not _drive_ready():
        return False, "--driveë¡ ì¤ííê³  íì¬/ëª©í ì°¨ë¡ë¥¼ ê°ê² í ë¤ ì¤ì  ê²½ê³ì  2ê°ê° ê²ì¶ë  ë ììíì¸ì."
    with control_lock:
        with state_lock:
            state["drive_enabled"] = enabled
            state["steering_enabled"] = enabled
            if not enabled:
                state["speed_cmd"] = 0
                state["steer_deg"] = 0.0
        if not enabled and steering_hardware is not None:
            steering_hardware.stop()
    return True, None


@app.route("/api/drive", methods=["POST"])
def api_drive():
    payload = request.get_json(force=True, silent=True) or {}
    enabled = payload.get("enabled") is True
    ok, message = _set_drive(enabled)
    if not ok:
        return jsonify({"ok": False, "message": message}), 409
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
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        print("Camera released. Server stopped.")


if __name__ == "__main__":
    main()
