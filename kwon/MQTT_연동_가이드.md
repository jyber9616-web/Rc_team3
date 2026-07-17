# MQTT 연동 가이드 (주영찬 · 박재혁용)

RC Team 3 프로젝트에서 차량(A/B/C) 쪽 코드가 백엔드와 통신하려면 MQTT 브로커에 붙어야 합니다.
설치부터 실제 코드 작성 스펙까지 순서대로 정리했습니다. 위에서부터 순서대로만 따라오면 됩니다.

---

## 0. 지금 상황 요약

- MQTT 브로커는 **권신용 PC**에서 돌아가고 있음 (다른 사람이 설치할 필요 없음)
- 브로커 주소는 **네트워크가 바뀔 때마다 같이 바뀜** — 시작 전에 권신용한테 오늘 쓸 최신 IP를 꼭 확인할 것
- 현재 기준 (바뀌면 이 문서도 업데이트):
  - `BROKER_ADDRESS = "172.20.10.5"`
  - `PORT = 1883`
  - 인증 없음 (아이디/비밀번호 필요 없음)

---

## 1. 설치할 것

### 1-1. Python 패키지 (필수)
```bash
pip install paho-mqtt
```

### 1-2. 커맨드라인 테스트 도구 (강력 추천 — 코드 짜기 전에 연결부터 확인할 때 씀)

**Windows:** [mosquitto.org/download](https://mosquitto.org/download/) 에서 Windows용 설치 (mosquitto_pub / mosquitto_sub 포함)

**Mac:**
```bash
brew install mosquitto
```

브로커 자체를 설치하는 게 아니라 **클라이언트 도구만** 설치하는 겁니다 — 설치 후 브로커 서비스가 자동 시작되면 그냥 꺼두면 됩니다 (안 써도 무방).

---

## 2. 네트워크 연결

- 권신용 핫스팟(또는 그날 쓰기로 한 공유 네트워크)에 접속
- **오토카(SODA 보드) 자체도 같은 네트워크에 붙어있어야** 실제 차량 통신이 됩니다. 노트북만 붙어있고 차량은 다른 네트워크면 노트북에서 하는 테스트는 되는데 실제 차량 코드는 안 됩니다.

---

## 3. 연결 테스트 (코드 작성 전에 반드시 먼저 할 것)

### 3-1. 포트가 열려있는지 확인

**Windows PowerShell:**
```powershell
Test-NetConnection -ComputerName 172.20.10.5 -Port 1883
```
`TcpTestSucceeded : True`가 나와야 정상. `False`면 같은 네트워크에 붙어있는지부터 다시 확인.

**Mac/Linux:**
```bash
nc -zv 172.20.10.5 1883
```

### 3-2. 실제 메시지 주고받기 테스트

**받는 쪽 (권신용이 미리 대기):**
```bash
mosquitto_sub -h localhost -t hello -v
```

**보내는 쪽 (본인 컴퓨터에서):**
```bash
mosquitto_pub -h 172.20.10.5 -t hello -m "test_from_영찬"
```

> ⚠️ **가장 흔한 실수:** `-h`에 `localhost`를 쓰면 안 됩니다. `localhost`는 "지금 이 명령어를 치는 바로 이 컴퓨터"를 가리켜서, 본인 컴퓨터에는 브로커가 없으니 무조건 실패합니다. **반드시 브로커가 있는 권신용 PC의 실제 IP**를 넣어야 합니다. (이거 하나 때문에 몇 시간 삽질했습니다 — 꼭 기억해주세요.)

성공하면 권신용 터미널에 메시지가 뜹니다. 여기까지 되면 이제 진짜 코드 작성하면 됩니다.

---

## 4. 코드에 쓸 메시지 스펙

### 토픽 구조
| 토픽 | 방향 | 용도 |
|---|---|---|
| `rcteam3/autocar/{car_id}/status` | 차량 → 백엔드 | 주기적 상태 발행 (5~10Hz) |
| `rcteam3/autocar/{car_id}/event` | 차량 → 백엔드 | 이벤트 발생 시 1회성 발행 |
| `rcteam3/autocar/{car_id}/command` | 백엔드 → 이 차량 | 개별 명령 (구독) |
| `rcteam3/autocar/all/command` | 백엔드 → 전체 차량 | 전체 명령 (구독) |

`{car_id}`는 본인이 담당하는 차량 값 (`A` / `B` / `C` 중 배정된 값)

### status 메시지 형식
```json
{
  "car_id": "A",
  "timestamp": 1720935600.123,
  "mode": "driving",
  "speed": 15,
  "steering_angle": -3,
  "distance_to_front": null,
  "brake": false
}
```
- `mode`: `"idle"` / `"driving"` / `"stopped"` / `"emergency_stop"` 중 하나
- `distance_to_front`: 선두차량(A)이면 항상 `null`, 후미차량(B, C)이면 LiDAR 실측 거리값
- `speed`, `steering_angle`은 실제 모터/조향 제어에 쓰는 값 그대로

### event 메시지 형식
```json
{
  "car_id": "B",
  "timestamp": 1720935601.5,
  "event_type": "obstacle_detected",
  "detail": {"object_type": "car", "confidence": 0.91}
}
```
`event_type` 종류:
- `lane_change` — detail: `{"direction": "left"/"right"}`
- `emergency_stop` — detail: `{"reason": "..."}`
- `obstacle_detected` — detail: `{"object_type": ..., "confidence": ...}`
- `brake` — detail: `{}`

### command 수신 형식 (구독해서 받는 쪽)
```json
{ "command": "start", "timestamp": 1720935590.0 }
```
`command` 종류: `start`, `stop`, `emergency_stop` — 받으면 그에 맞게 주행 시작/정지/긴급정지 동작

### 연결 끊김 대비 LWT (필수)
차량이 갑자기 꺼지거나 통신이 끊기면 백엔드가 자동으로 알 수 있도록, 접속할 때 아래처럼 반드시 등록:
```python
client.will_set(
    f"rcteam3/autocar/{car_id}/status",
    payload=json.dumps({"car_id": car_id, "mode": "offline"}),
    qos=1,
    retain=True
)
```

---

## 5. 최소 예제 코드 (뼈대)

```python
import json
import time
import paho.mqtt.client as mqtt

BROKER_ADDRESS = "172.20.10.5"  # 오늘 쓸 최신 IP로 권신용한테 확인
PORT = 1883
CAR_ID = "A"  # 본인 담당 차량으로 변경

STATUS_TOPIC = f"rcteam3/autocar/{CAR_ID}/status"
EVENT_TOPIC = f"rcteam3/autocar/{CAR_ID}/event"
COMMAND_TOPIC = f"rcteam3/autocar/{CAR_ID}/command"
COMMAND_ALL_TOPIC = "rcteam3/autocar/all/command"


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("브로커 연결 성공")
        client.subscribe(COMMAND_TOPIC)
        client.subscribe(COMMAND_ALL_TOPIC)
    else:
        print(f"연결 실패, 코드: {rc}")


def on_message(client, userdata, msg):
    data = json.loads(msg.payload.decode("utf-8"))
    command = data.get("command")
    if command == "start":
        pass  # TODO: 주행 시작
    elif command == "stop":
        pass  # TODO: 주행 정지
    elif command == "emergency_stop":
        pass  # TODO: 긴급 정지


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# 연결 끊김 대비 LWT 등록 (connect 전에 설정해야 함)
client.will_set(
    STATUS_TOPIC,
    payload=json.dumps({"car_id": CAR_ID, "mode": "offline"}),
    qos=1,
    retain=True,
)

client.connect(BROKER_ADDRESS, PORT, 60)
client.loop_start()

# 상태를 주기적으로 발행하는 예시 (실제 값으로 교체)
try:
    while True:
        status = {
            "car_id": CAR_ID,
            "timestamp": time.time(),
            "mode": "driving",
            "speed": 0,             # TODO: 실제 속도 값
            "steering_angle": 0,    # TODO: 실제 조향각 값
            "distance_to_front": None,  # A차량은 항상 None, B/C는 LiDAR 값
            "brake": False,
        }
        client.publish(STATUS_TOPIC, json.dumps(status), qos=0)
        time.sleep(0.1)  # 10Hz
except KeyboardInterrupt:
    client.loop_stop()
    client.disconnect()
```

이벤트(차선변경/장애물/비상정지/브레이크)는 상태와 별개로, 발생하는 순간에만 한 번 발행하면 됩니다:
```python
event = {
    "car_id": CAR_ID,
    "timestamp": time.time(),
    "event_type": "obstacle_detected",
    "detail": {"object_type": "car", "confidence": 0.91},
}
client.publish(EVENT_TOPIC, json.dumps(event), qos=1)
```

### 참고할 수 있는 실제 예제 파일
- `kwon/Backend_Dash/dummy_publisher.py` — status 발행하는 최소 예제
- `kwon/Backend_Dash/dummy_event_publisher.py` — event 발행하는 최소 예제
- `lee/HMI_UI/voice_alert.py` — event 구독 + 파싱하는 최소 예제

---

## 6. 자주 하는 실수 체크리스트

- [ ] `-h localhost`로 원격 브로커 테스트 → **반드시 실제 브로커 IP 사용**
- [ ] 본인 컴퓨터/오토카가 권신용 핫스팟과 **다른 네트워크**에 붙어있음 → 접속 안 됨, 네트워크부터 확인
- [ ] 브로커 IP를 예전 값 그대로 씀 → 네트워크 바뀔 때마다 바뀌니 오늘 시작 전에 최신 값 확인
- [ ] `client.will_set()`을 `client.connect()` **이후에** 호출함 → LWT는 반드시 connect 전에 설정해야 적용됨
- [ ] status를 너무 빠르게(예: 0.01초 간격) 발행함 → 5~10Hz(0.1~0.2초 간격)면 충분

---

## 7. 문제 생기면

- 브로커/네트워크 연결 문제: 권신용한테 문의
- 오늘의 최신 브로커 IP: 단톡방에서 확인
