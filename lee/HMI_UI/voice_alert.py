"""
voice_alert.py — TTS 음성 알림 (RC Team 3)

MQTT 브로커에 직접 붙어서 모든 차량(A, B, C)의 event 토픽을 구독하고,
event_type에 따라 음성 안내 문구를 출력한다.

지금은 print()로 문구만 출력하는 상태 — 실제 TTS 엔진(gTTS, pyttsx3 등)을
붙이려면 speak() 함수 안의 print를 TTS 호출로 바꾸면 됨.
"""

import paho.mqtt.client as mqtt
import json

BROKER_ADDRESS = "172.20.10.5"  # 브로커 PC의 Wi-Fi IP (재연결 시 바뀔 수 있음)
PORT = 1883
EVENT_TOPIC_FILTER = "rcteam3/autocar/+/event"  # 차량 전체(A/B/C) 이벤트 구독


def speak(text: str):
    """실제 TTS 엔진을 붙일 자리. 지금은 콘솔 출력으로 대체."""
    print(f"🔊 [TTS]: {text}")


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("✅ [TTS 시스템] 브로커에 연결되었습니다! 음성 안내 대기 중...")
        client.subscribe(EVENT_TOPIC_FILTER)
    else:
        print(f"❌ 연결 실패! 코드: {rc}")


def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode("utf-8"))
    except json.JSONDecodeError:
        print(f"⚠️ JSON 파싱 실패: {msg.payload!r}")
        return

    car_id = data.get("car_id", "?")
    event_type = data.get("event_type")
    detail = data.get("detail", {})

    if event_type == "brake":
        speak(f"{car_id} 차량, 전방 충돌 위험. 긴급 제동합니다.")
    elif event_type == "lane_change":
        direction = detail.get("direction")
        direction_kr = {"left": "왼쪽", "right": "오른쪽"}.get(direction, direction)
        speak(f"{car_id} 차량, {direction_kr}으로 차선을 변경합니다.")
    elif event_type == "emergency_stop":
        reason = detail.get("reason", "")
        speak(f"{car_id} 차량, 비상 정지합니다. {reason}".strip())
    elif event_type == "obstacle_detected":
        obj_type = detail.get("object_type", "장애물")
        speak(f"{car_id} 차량, 전방에 {obj_type} 감지.")
    else:
        # 알 수 없는 event_type은 조용히 무시 (로그만 남김)
        print(f"[알 수 없는 이벤트] {data}")


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER_ADDRESS, PORT, 60)
client.loop_forever()