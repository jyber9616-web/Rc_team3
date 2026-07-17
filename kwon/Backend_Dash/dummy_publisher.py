import paho.mqtt.client as mqtt
import json
import time
import random

# 1. 공개 우체국 주소 사용 (내 PC에 프로그램 설치할 필요 없음!)
BROKER_ADDRESS = "10.2.105.65"
BROKER_ADDRESS = "127.0.0.1" ## 권신용 컴퓨터 주소에요
PORT = 1883

# 2. 다른 사람의 데이터와 섞이지 않게 우리 팀만의 고유한 방(Topic) 이름을 만듭니다.
TOPIC = "rcteam3/autocar/status"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("✅ 공개 MQTT 우체국(test.mosquitto.org)에 성공적으로 연결되었습니다!")
    else:
        print(f"❌ 연결 실패! 에러 코드: {rc}")

client = mqtt.Client()
client.on_connect = on_connect
client.connect(BROKER_ADDRESS, PORT, 60)

# 백그라운드에서 네트워크 트래픽 처리 시작
client.loop_start()

print("🚗 가짜(Dummy) 데이터 전송을 시작합니다... (종료하려면 터미널에서 Ctrl+C)")

try:
    while True:
        dummy_data = {
            "car_id": "A",
            "speed": random.randint(10, 20),
            "steering_angle": random.randint(-5, 5),
            "current_lane": 2,
            "distance_to_front": None,
            "brake": random.choice([True, False, False])
        }

        # 파이썬 딕셔너리를 JSON 문자열로 변환하여 전송
        message = json.dumps(dummy_data)
        client.publish(TOPIC, message)
        print(f"📡 전송 완료: {message}")
        
        time.sleep(1)

except KeyboardInterrupt:
    print("\n🛑 데이터 전송을 종료합니다.")
    client.loop_stop()
    client.disconnect()