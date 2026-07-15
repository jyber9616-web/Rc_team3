"""
mqtt_broker.py — MQTT 클라이언트 로직 (RC Team 3 backend)

FastAPI(main.py)에서 이 모듈을 import해서 사용.
이 파일 자체는 MQTT 연결/구독/발행만 담당하고, WebSocket이나 FastAPI는 모른다.
데이터가 들어오면 main.py가 등록해둔 콜백(on_state_update, on_event)을 호출해서 넘겨준다.
"""

import json
import time
from typing import Callable, Dict, Optional

import paho.mqtt.client as mqtt

# ── 설정 ──────────────────────────────────────────────────────────
BROKER_ADDRESS = "10.2.105.65"  # Mosquitto 브로커 IP (Wi-Fi 재연결 시 바뀔 수 있음)
BROKER_PORT = 1883

CAR_IDS = ["A", "B", "C"]

STATUS_TOPIC_FILTER = "rcteam3/autocar/+/status"
EVENT_TOPIC_FILTER = "rcteam3/autocar/+/event"
COMMAND_TOPIC_TEMPLATE = "rcteam3/autocar/{car_id}/command"
COMMAND_ALL_TOPIC = "rcteam3/autocar/all/command"

VALID_COMMANDS = {"start", "stop", "emergency_stop"}

# ── 상태 저장소 (car_id -> 최신 status dict) ─────────────────────
car_state: Dict[str, dict] = {car_id: {"car_id": car_id, "mode": "offline"} for car_id in CAR_IDS}

# main.py가 여기에 콜백을 등록하면, 새 데이터가 올 때마다 호출됨
on_state_update: Optional[Callable[[Dict[str, dict]], None]] = None
on_event: Optional[Callable[[dict], None]] = None


def _on_connect(client: mqtt.Client, userdata, flags, rc):
    if rc == 0:
        print("[MQTT] 브로커 연결 성공")
        client.subscribe(STATUS_TOPIC_FILTER)
        client.subscribe(EVENT_TOPIC_FILTER)
        print(f"[MQTT] 구독: {STATUS_TOPIC_FILTER}, {EVENT_TOPIC_FILTER}")
    else:
        print(f"[MQTT] 연결 실패, code={rc}")


def _on_disconnect(client: mqtt.Client, userdata, rc):
    print(f"[MQTT] 연결 끊김 (code={rc}), 재연결 시도 중...")


def _on_message(client: mqtt.Client, userdata, msg: mqtt.MQTTMessage):
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
    except json.JSONDecodeError:
        print(f"[MQTT] JSON 파싱 실패: topic={msg.topic}, payload={msg.payload!r}")
        return

    # 토픽 형식: rcteam3/autocar/{car_id}/status 또는 .../event
    parts = msg.topic.split("/")
    if len(parts) != 4:
        print(f"[MQTT] 예상 못한 토픽 형식: {msg.topic}")
        return

    _, _, car_id, msg_type = parts
    if car_id not in CAR_IDS:
        print(f"[MQTT] 알 수 없는 car_id: {car_id}")
        return

    if msg_type == "status":
        car_state[car_id] = payload
        if on_state_update is not None:
            on_state_update(car_state)
    elif msg_type == "event":
        print(f"[MQTT] 이벤트 수신: {payload}")
        if on_event is not None:
            on_event(payload)


client = mqtt.Client()
client.on_connect = _on_connect
client.on_disconnect = _on_disconnect
client.on_message = _on_message


def start():
    """서버 시작 시 호출. 브로커가 지금 꺼져있어도 예외 없이 백그라운드에서 재연결 시도함."""
    client.reconnect_delay_set(min_delay=1, max_delay=30)
    client.connect_async(BROKER_ADDRESS, BROKER_PORT, keepalive=60)
    client.loop_start()
    print("[MQTT] 연결 시도 시작")


def stop():
    client.loop_stop()
    client.disconnect()
    print("[MQTT] 연결 종료")


def publish_command(command: str, target: str = "all"):
    """대시보드에서 받은 명령을 차량으로 재발행"""
    if command not in VALID_COMMANDS:
        print(f"[CMD] 잘못된 명령 무시: {command}")
        return

    payload = json.dumps({"command": command, "timestamp": time.time()})

    if target == "all":
        client.publish(COMMAND_ALL_TOPIC, payload, qos=1)
        print(f"[CMD] 전체 차량에 '{command}' 전송")
    elif target in CAR_IDS:
        topic = COMMAND_TOPIC_TEMPLATE.format(car_id=target)
        client.publish(topic, payload, qos=1)
        print(f"[CMD] {target} 차량에 '{command}' 전송")
    else:
        print(f"[CMD] 잘못된 target 무시: {target}")