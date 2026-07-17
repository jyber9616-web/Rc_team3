<!-- 1. 주영찬 · 박재혁 — Core Driving & AI 파트용 프롬프트

나는 RC카 자율 군집주행 프로젝트의 "선두 차량(A) 및 후미 차량(B, C) 주행/인식" 담당이야.
차량은 한백전자 AutoCar Prime, NVIDIA 기반 SODA 보드에서 Python으로 개발해.

내 작업 범위:
- 선두 차량 A: 카메라 기반 CNN 라인 트레이싱으로 차선 추종, 필요 시 차선 변경 판단/수행
- 후미 차량 B, C: YOLO로 전방 차량·장애물 인식 + 360도 LiDAR로 앞차와의 거리 측정, 목표 차간거리 유지하도록 속도/조향 제어
- 계산한 속도, 조향각, 차간거리, 이벤트(차선변경/장애물/비상정지/브레이크)를 MQTT로 백엔드에 전송

브로커 주소: 172.20.10.5:1883 (paho-mqtt 사용)

토픽 구조:
- rcteam3/autocar/{car_id}/status  → 주기적 상태 (5~10Hz)
- rcteam3/autocar/{car_id}/event   → 1회성 이벤트 발생 시
- rcteam3/autocar/{car_id}/command → 백엔드가 이 차량한테 보내는 개별 명령 (구독)
- rcteam3/autocar/all/command      → 백엔드가 전체 차량한테 보내는 명령 (구독)
{car_id}는 "A", "B", "C" 중 내가 담당하는 차량 값.

status 메시지 형식 (예시, car_id=A):
{
  "car_id": "A",
  "timestamp": 1720935600.123,
  "mode": "driving",
  "speed": 15,
  "steering_angle": -3,
  "distance_to_front": null,
  "brake": false
}
- mode는 "idle" / "driving" / "stopped" / "emergency_stop" 중 하나
- distance_to_front는 선두차량(A)이면 항상 null, 후미차량(B, C)이면 LiDAR로 측정한 실제 거리값
- speed, steering_angle 단위는 우리가 실제 모터/조향 제어에 쓰는 값 그대로 사용

event 메시지 형식:
{
  "car_id": "B",
  "timestamp": 1720935601.500,
  "event_type": "obstacle_detected",
  "detail": {"object_type": "car", "confidence": 0.91}
}
event_type 종류: lane_change (detail: {"direction": "left"/"right"}), emergency_stop (detail: {"reason": "..."}), obstacle_detected (detail: {"object_type": ..., "confidence": ...}), brake (detail: {})

command 메시지 형식 (구독해서 받는 쪽):
{ "command": "start", "timestamp": 1720935590.000 }
command 종류: start, stop, emergency_stop — 받으면 그에 맞게 주행 시작/정지/긴급정지 동작

연결 끊김 대비 LWT도 등록해야 함:
client.will_set(f"rcteam3/autocar/{car_id}/status", payload=json.dumps({"car_id": car_id, "mode": "offline"}), qos=1, retain=True)

이 스펙에 맞춰서 내 차량의 인식/제어 로직에 MQTT publish/subscribe 코드를 붙여줘. 실제 인식·제어 알고리즘(CNN, YOLO, LiDAR 처리)은 내가 이미 만들고 있으니, 그 결과값을 위 형식대로 MQTT로 보내고 command를 받아 처리하는 부분을 중심으로 도와줘.


2. 이명연 — Hardware UI & Dashboard 파트용 프롬프트

나는 RC카 자율 군집주행 프로젝트의 "차량 LED 상태 표시 + 통합 대시보드" 담당이야.

내 작업 범위:
1. 차량 LED 제어 (SODA 보드, GPIO 또는 라이브러리 사용)
   - 급감속 시 브레이크등
   - 차선 변경 시 방향지시등 (좌/우)
   - 비상 정지 시 양쪽 비상 점멸등
   이 정보는 MQTT event 토픽에서 받아서 트리거해야 해.

2. React 기반 통합 대시보드 (PyWebView로 데스크톱 앱 실행)
   - 차량 A, B, C의 상태를 한 화면에 카드 3개로 표시 (페이지 분리 아님)
   - 각 카드에 연결 상태, 주행 모드, 속도, 조향각, 차간거리, 인식 상태 표시
   - 실시간 차간거리 그래프, 시스템 로그 표시
   - 차량별 시작/정지 버튼 + 전체 시작/정지/긴급정지 버튼
   - 데이터는 백엔드(FastAPI)가 WebSocket으로 실시간 전달해줌 (내가 MQTT에 직접 붙는 게 아니라 WebSocket만 연결하면 됨)

백엔드에서 오는 WebSocket 메시지는 차량별로 아래 형식을 모아서 보내줄 예정:
{
  "A": {"car_id": "A", "timestamp": ..., "mode": "driving", "speed": 15, "steering_angle": -3, "distance_to_front": null, "brake": false},
  "B": {...},
  "C": {...}
}

이벤트(차선변경/장애물/비상정지/브레이크)는 별도로 아래 형식으로 옴:
{"car_id": "B", "timestamp": ..., "event_type": "obstacle_detected", "detail": {"object_type": "car", "confidence": 0.91}}
event_type 종류: lane_change, emergency_stop, obstacle_detected, brake

버튼으로 명령을 보낼 때는 WebSocket을 통해 백엔드에 아래 형식으로 요청하면, 백엔드가 MQTT로 차량에 전달해줌:
{ "command": "start", "target": "all" }  또는 target을 "A"/"B"/"C"로 특정 가능
command 종류: start, stop, emergency_stop

이 스펙에 맞춰서 (1) LED 제어용 MQTT event 구독 코드, (2) React 대시보드에서 WebSocket 연결 + 차량 3대 카드 UI + 차간거리 그래프 + 로그 + 제어 버튼을 만들어줘. PyWebView로 감싸서 데스크톱 앱으로 실행하는 부분도 포함해줘.


3. 권신용 — Backend·통신 인프라 파트용 프롬프트 (본인용)

나는 RC카 자율 군집주행 프로젝트의 "MQTT 브로커 + 백엔드(FastAPI) + 차량 간 통신" 담당이야.
MQTT 브로커는 이미 내 PC(Mosquitto)에 설정 완료했고, 팀원들과 아래 메시지 스펙도 확정했어.

토픽 구조:
- rcteam3/autocar/{car_id}/status  (car_id: A, B, C) — 차량 → 백엔드, 5~10Hz
- rcteam3/autocar/{car_id}/event   — 차량 → 백엔드, 이벤트 발생 시
- rcteam3/autocar/{car_id}/command — 백엔드 → 개별 차량
- rcteam3/autocar/all/command      — 백엔드 → 전체 차량

status 예시:
{"car_id": "A", "timestamp": 1720935600.123, "mode": "driving", "speed": 15, "steering_angle": -3, "distance_to_front": null, "brake": false}

event 예시:
{"car_id": "B", "timestamp": 1720935601.5, "event_type": "obstacle_detected", "detail": {"object_type": "car", "confidence": 0.91}}

command 예시:
{"command": "start", "timestamp": 1720935590.0}

내가 만들어야 하는 것:
1. FastAPI 서버에서 paho-mqtt 클라이언트로 rcteam3/autocar/+/status 와 rcteam3/autocar/+/event 를 와일드카드 구독
2. 받은 데이터를 차량별(A/B/C)로 메모리에 최신 상태 저장 (딕셔너리)
3. 상태가 갱신될 때마다 연결된 모든 WebSocket 클라이언트(대시보드)에 {"A": {...}, "B": {...}, "C": {...}} 형태로 broadcast
4. event는 별도로 WebSocket에 실시간 push (로그/TTS/LED 트리거용)
5. 대시보드에서 WebSocket으로 들어오는 제어 명령({"command": "start", "target": "all"} 등)을 받아서 해당 MQTT command 토픽으로 재발행
6. 각 차량의 LWT(연결 끊김) 처리 — status의 mode가 "offline"으로 오면 프론트에 표시
7. tmux로 이 FastAPI 서버, MQTT 브로커 상태를 세션별로 관리할 수 있게 실행 스크립트도 구성

FastAPI + paho-mqtt + WebSocket을 사용한 브릿지 서버 코드를 이 스펙에 맞춰서 작성해줘. MQTT 구독 로직과 WebSocket broadcast 로직이 비동기로 잘 맞물리게 해줘 (FastAPI는 async 프레임워크라 paho-mqtt의 동기 콜백을 어떻게 연결할지도 신경써줘). -->