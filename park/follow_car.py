# 차량 객체추종

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

import tensorflow as tf
import paho.mqtt.client as mqtt

gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
    try:
        tf.config.experimental.set_memory_growth(gpus[0], True)
        print("✅ GPU 메모리 동적 할당 완료")
    except RuntimeError as e:
        print(e)

# 3. 라이브러리 불러오기
from pop import Pilot

# ── MQTT 설정 (kwon/MQTT_연동_가이드.md 참고) ────────────────────────
BROKER_ADDRESS = "172.20.10.5"  # 신용이 브로커 주소, 네트워크 바뀌면 여기 수정
PORT = 1883
CAR_ID = "A"  # ⚠️ 담당 차량으로 맞출 것 (A/B/C)

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


try:
    if 'cam' in locals(): cam.stop(); del cam
    if 'ac' in locals(): ac.stop()
except Exception:
    pass

cam = Pilot.Camera(width=320, height=320)
ac = Pilot.AutoCar()
OF = Pilot.Object_Follow(cam)
OF.load_model()

# 💡 [핵심 보완] 옛날 이미지 찌꺼기 털어내기
# 카메라를 켜자마자 바로 화면을 띄우지 않고, 버퍼에 고여있던 옛날 프레임을 10장쯤 흘려보냅니다.
print("🧹 카메라 버퍼의 예전 이미지 찌꺼기를 청소하는 중...")
for _ in range(10):
    try:
        if hasattr(cam, 'read_image'): cam.read_image()
        elif hasattr(cam, 'np_array'): cam.np_array()
    except Exception:
        pass
    time.sleep(0.05)

# 📺 이제 깨끗해진 채널로 뷰어를 켭니다.
OF.show()

STOP_SIZE_RATE = 0.10
STATUS_PUBLISH_INTERVAL = 0.1  # 10Hz
last_status_time = 0.0
was_tracking = False
was_backing_up = False

print("🚗 오토카 객체 추종 주행을 시작합니다!")

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

        v = OF.detect(index='car')

        if v is not None and isinstance(v, dict) and 'box' in v:
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
                print(f"🏃 목표 차량 추적 중 (전진)... 크기 비율: {v['size_rate']:.2f}  ", end='\r', flush=True)
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
            mode = "driving"  # 대상을 찾는 중이지만 여전히 주행 세션 상태
            brake = False
            was_tracking = False
            was_backing_up = False
            print("🔍 차량을 찾는 중입니다...                           ", end='\r', flush=True)

        now = time.time()
        if now - last_status_time >= STATUS_PUBLISH_INTERVAL:
            publish_status(mode, speed, ac.steering, brake)
            last_status_time = now

        time.sleep(0.05)

except (KeyboardInterrupt, SystemExit, Exception) as e:
    print(f"\n🛑 주행이 중단되었습니다. (원인: {type(e).__name__} - {e})")

finally:
    print("🧹 오토카 자원 및 카메라 뷰를 초기화하는 중...")
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
