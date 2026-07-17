"""
dummy_event_publisher.py — voice_alert.py 테스트용 가짜 이벤트 발행기

brake, lane_change, emergency_stop, obstacle_detected 이벤트를
3초 간격으로 순서대로 하나씩 브로커에 발행한다.

voice_alert.py를 먼저 켜놓고 이 스크립트를 실행하면,
voice_alert.py 콘솔에 4개의 TTS 문구가 순서대로 뜨는지 확인하면 된다.
"""

import paho.mqtt.client as mqtt
import json
import time

BROKER_ADDRESS = "172.20.10.5"
PORT = 1883
CAR_ID = "A"
EVENT_TOPIC = f"rcteam3/autocar/{CAR_ID}/event"


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("브로커 연결 성공, 테스트 이벤트 전송 시작")
    else:
        print(f"연결 실패, 코드: {rc}")


client = mqtt.Client()
client.on_connect = on_connect
client.connect(BROKER_ADDRESS, PORT, 60)
client.loop_start()

time.sleep(1)  # 연결될 시간을 잠깐 줌

test_events = [
    {"car_id": CAR_ID, "event_type": "brake", "detail": {}},
    {"car_id": CAR_ID, "event_type": "lane_change", "detail": {"direction": "left"}},
    {"car_id": CAR_ID, "event_type": "obstacle_detected", "detail": {"object_type": "car", "confidence": 0.91}},
    {"car_id": CAR_ID, "event_type": "emergency_stop", "detail": {"reason": "obstacle"}},
]

for event in test_events:
    payload = json.dumps(event)
    client.publish(EVENT_TOPIC, payload, qos=1)
    print(f"전송: {payload}")
    time.sleep(3)

print("테스트 이벤트 4개 전송 완료")
client.loop_stop()
client.disconnect()
