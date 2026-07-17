# 차량 후미 파란색 테이프 마커 추종
#
# Pilot.Object_Follow의 사전학습 모델이 검정색 차체를 'car'로 잘 못 잡아서
# (엉뚱하게 'coke' 등으로 인식), 차량 후미에 붙인 파란색 테이프를
# OpenCV 색상 검출로 직접 찾아 추종하는 방식.

# 1. 🚨 무조건 '가장 먼저' 실행
import sys
import os

try:
    import ctypes
    ctypes.CDLL('/usr/lib/aarch64-linux-gnu/libgomp.so.1', mode=ctypes.RTLD_GLOBAL)
    print("✅ 시스템 TLS 메모리 우회 블록 주입 성공!")
except Exception as e:
    print(f"⚠️ 주입 시도 중 알림: {e}")

# 2. 모듈 로드
import json
import threading
import time

import cv2
import numpy as np
import paho.mqtt.client as mqtt
import ipywidgets
from IPython.display import display

# 3. 라이브러리 불러오기
from pop import Pilot

# ── MQTT 설정 (kwon/MQTT_연동_가이드.md 참고) ────────────────────────
BROKER_ADDRESS = "172.20.10.5"  # 신용이 브로커 주소, 네트워크 바뀌면 여기 수정
PORT = 1883
CAR_ID = "B"  # 명연이 차량 (A=영찬 차선추종, B=명연, C=재혁 — 이 파일은 파란색 추종용)

STATUS_TOPIC = f"rcteam3/autocar/{CAR_ID}/status"
EVENT_TOPIC = f"rcteam3/autocar/{CAR_ID}/event"
COMMAND_TOPIC = f"rcteam3/autocar/{CAR_ID}/command"
COMMAND_ALL_TOPIC = "rcteam3/autocar/all/command"

_emergency_stop = threading.Event()
_paused = threading.Event()


def _on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("[MQTT] 브로커 연결 성공")
        client.subscribe(COMMAND_TOPIC)
        client.subscribe(COMMAND_ALL_TOPIC)
    else:
        print(f"[MQTT] 연결 실패, code={rc}")


def _on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode("utf-8"))
    except json.JSONDecodeError:
        return

    command = data.get("command")
    if command == "emergency_stop":
        _emergency_stop.set()
        publish_event("emergency_stop", {"reason": "dashboard"})
        print("\n[MQTT] 긴급 정지 명령 수신!")
    elif command == "stop":
        _paused.set()
        print("\n[MQTT] 정지 명령 수신")
    elif command == "start":
        _paused.clear()
        _emergency_stop.clear()
        print("\n[MQTT] 시작 명령 수신")


mqtt_client = mqtt.Client()
mqtt_client.on_connect = _on_connect
mqtt_client.on_message = _on_message
mqtt_client.will_set(
    STATUS_TOPIC,
    payload=json.dumps({"car_id": CAR_ID, "mode": "offline"}),
    qos=1,
    retain=True,
)
mqtt_client.connect(BROKER_ADDRESS, PORT, 60)
mqtt_client.loop_start()


def publish_status(mode, speed, steering_angle, brake):
    payload = {
        "car_id": CAR_ID,
        "timestamp": time.time(),
        "mode": mode,
        "speed": speed,
        "steering_angle": round(steering_angle, 2),
        "distance_to_front": None,  # 선두 차량은 항상 None
        "brake": brake,
    }
    mqtt_client.publish(STATUS_TOPIC, json.dumps(payload), qos=0)


def publish_event(event_type, detail=None):
    payload = {
        "car_id": CAR_ID,
        "timestamp": time.time(),
        "event_type": event_type,
        "detail": detail or {},
    }
    mqtt_client.publish(EVENT_TOPIC, json.dumps(payload), qos=1)


# ── 파란색 테이프 마커 검출 ────────────────────────────────────────
# ⚠️ 실제 테이프 색/조명에 따라 이 HSV 범위를 조정해야 할 수 있음
BLUE_LOWER = np.array([100, 100, 50])
BLUE_UPPER = np.array([130, 255, 255])
MIN_AREA_RATIO = 0.005  # 화면 전체 대비 이 비율보다 작으면 "못 찾음"으로 취급


def detect_blue_marker(frame):
    """프레임에서 파란 테이프 마커를 찾아 중심 x(-1~1)와 크기 비율을 반환. 못 찾으면 None."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    largest = max(contours, key=cv2.contourArea)
    frame_h, frame_w = frame.shape[:2]
    area = cv2.contourArea(largest)
    size_rate = area / (frame_w * frame_h)

    if size_rate < MIN_AREA_RATIO:
        return None

    x, y, w, h = cv2.boundingRect(largest)
    center_x = x + w / 2
    norm_x = (center_x - frame_w / 2) / (frame_w / 2)  # -1(왼쪽) ~ 1(오른쪽)

    return {"x": norm_x, "size_rate": size_rate, "box": (x, y, w, h)}


def read_frame(camera):
    """camera에서 프레임을 BGR numpy 배열로 읽어온다 (follow_auto_car.py와 동일한 방식)."""
    raw_img = camera.read() if hasattr(camera, 'read') else camera()
    if raw_img is None:
        return None

    if hasattr(raw_img, 'value') and not isinstance(raw_img, np.ndarray):
        img_bytes = raw_img.value
        if isinstance(img_bytes, bytes) and len(img_bytes) > 0:
            nparr = np.frombuffer(img_bytes, dtype=np.uint8)
            return cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        return None

    return raw_img


try:
    if 'cam' in locals(): cam.stop(); del cam
    if 'ac' in locals(): ac.stop()
except Exception:
    pass

cam = Pilot.Camera(width=320, height=320)
ac = Pilot.AutoCar()

# 💡 옛날 이미지 찌꺼기 털어내기
print("🧹 카메라 버퍼의 예전 이미지 찌꺼기를 청소하는 중...")
for _ in range(10):
    read_frame(cam)
    time.sleep(0.05)

# 📺 실시간 뷰어 — 내가 읽은 프레임을 위젯 하나에 계속 갱신 (cam.show()와 따로 프레임을 안 나눠 가짐)
view_widget = ipywidgets.Image(format='jpeg')
display(view_widget)

STOP_SIZE_RATE = 0.42  # 목표 추종 거리 기준 (이 크기 비율을 유지하려고 함)
STATUS_PUBLISH_INTERVAL = 0.1  # 10Hz
VIEW_UPDATE_INTERVAL = 0.1  # 이 간격으로만 위젯 갱신
last_view_update_time = 0.0
last_status_time = 0.0
was_tracking = False
was_backing_up = False

print("🚗 파란색 테이프 마커 추종 주행을 시작합니다!")

try:
    while True:
        if _emergency_stop.is_set():
            ac.stop()
            ac.steering = 0
            publish_status("emergency_stop", 0, 0, True)
            print("🚨 긴급 정지 상태 — MQTT 명령 대기 중...          ", end='\r', flush=True)
            time.sleep(0.2)
            continue

        if _paused.is_set():
            ac.stop()
            publish_status("stopped", 0, ac.steering, False)
            print("⏸️ 정지 상태 — MQTT 시작 명령 대기 중...          ", end='\r', flush=True)
            time.sleep(0.2)
            continue

        frame = read_frame(cam)
        v = detect_blue_marker(frame) if frame is not None else None

        now_view = time.time()
        if frame is not None and now_view - last_view_update_time >= VIEW_UPDATE_INTERVAL:
            ok, jpeg = cv2.imencode(".jpg", frame)
            if ok:
                view_widget.value = jpeg.tobytes()
            last_view_update_time = now_view

        if v is not None:
            steer = v['x'] * 4
            if steer > 1: steer = 1
            elif steer < -1: steer = -1
            ac.steering = steer

            if not was_tracking:
                publish_event("obstacle_detected", {"object_type": "car", "confidence": 0.9})
                was_tracking = True

            if v['size_rate'] < STOP_SIZE_RATE:
                ac.forward(50)
                speed = 50
                was_backing_up = False
                print(f"🏃 파란 마커 추적 중 (전진)... 크기 비율: {v['size_rate']:.2f}  ", end='\r', flush=True)
            else:
                ac.backward(50)
                speed = -50
                if not was_backing_up:
                    publish_event("brake")
                    was_backing_up = True
                print(f"🔙 너무 가깝습니다! 후진 중... 크기 비율: {v['size_rate']:.2f}  ", end='\r', flush=True)

            mode = "driving"
            brake = was_backing_up
        else:
            ac.stop()
            speed = 0
            mode = "driving"  # 마커를 찾는 중이지만 여전히 주행 세션 상태
            brake = False
            was_tracking = False
            was_backing_up = False
            print("🔍 파란 마커를 찾는 중입니다...                           ", end='\r', flush=True)

        now = time.time()
        if now - last_status_time >= STATUS_PUBLISH_INTERVAL:
            publish_status(mode, speed, ac.steering, brake)
            last_status_time = now

        time.sleep(0.05)

except (KeyboardInterrupt, SystemExit, Exception) as e:
    print(f"\n🛑 주행이 중단되었습니다. (원인: {type(e).__name__} - {e})")

finally:
    print("🧹 오토카 자원을 초기화하는 중...")
    try:
        ac.stop()
        ac.steering = 0
        publish_status("stopped", 0, 0, False)
        cam.stop()
        del cam
        print("✨ 초기화 완료!")
    except NameError:
        pass
    mqtt_client.loop_stop()
    mqtt_client.disconnect()
