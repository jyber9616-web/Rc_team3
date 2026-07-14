import paho.mqtt.client as mqtt
import json

# 방금 만든 '내 컴퓨터(로컬)' 우체국 주소를 사용합니다.
BROKER_ADDRESS = "10.2.105.65"
PORT = 1883
TOPIC = "rcteam3/autocar/status"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("✅ [TTS 시스템] 나만의 우체국에 연결되었습니다! 음성 안내 대기 중...")
        # 🚨 여기에 '구독(Subscribe)' 신청 한 줄이 꼭 필요합니다! 🚨
        client.subscribe(TOPIC)
    else:
        print("❌ 연결 실패!")

def on_message(client, userdata, msg):
    raw_data = msg.payload.decode("utf-8")
    data = json.loads(raw_data)
    
    # 브레이크 신호가 들어왔을 때만 텍스트 알림 출력
    if data.get("brake") == True:
        print("🔊 [TTS 시스템 작동]: '전방 충돌 위험. 긴급 제동합니다.'")
    else:
        # 정상 주행 중에는 조용히 데이터를 무시합니다.
        pass 

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER_ADDRESS, PORT, 60)
# 백그라운드에서 계속 대기하며 데이터가 올 때마다 on_message 함수를 실행합니다.
client.loop_forever()