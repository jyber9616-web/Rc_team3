"""
main.py — FastAPI 앱: WebSocket 대시보드 연동 (RC Team 3 backend)

역할:
1. 서버 시작 시 mqtt_broker.start()로 MQTT 연결
2. mqtt_broker에서 새 데이터가 오면 콜백을 통해 받아서 WebSocket으로 대시보드에 broadcast
3. 대시보드에서 오는 시작/정지/긴급정지 명령을 mqtt_broker.publish_command()로 전달

실행:
    uvicorn main:app --reload --host 0.0.0.0 --port 8000

대시보드는 ws://<이 서버 IP>:8000/ws 로 접속하면 됨.
"""

import asyncio
import json
from typing import Dict, Optional, Set

from fastapi import FastAPI, WebSocket, WebSocketDisconnect

import mqtt_broker

app = FastAPI(title="RC Team 3 MQTT-WebSocket Bridge")

connected_clients: Set[WebSocket] = set()
main_loop: Optional[asyncio.AbstractEventLoop] = None


# ── mqtt_broker 콜백 (MQTT 백그라운드 스레드에서 호출됨) ────────────
def _handle_state_update(car_state: Dict[str, dict]):
    if main_loop is not None:
        asyncio.run_coroutine_threadsafe(_broadcast_state(car_state), main_loop)


def _handle_event(event_payload: dict):
    if main_loop is not None:
        asyncio.run_coroutine_threadsafe(_broadcast_event(event_payload), main_loop)


mqtt_broker.on_state_update = _handle_state_update
mqtt_broker.on_event = _handle_event


# ── WebSocket broadcast (메인 이벤트 루프에서 실행됨) ───────────────
async def _broadcast(message: str):
    dead_clients = set()
    for ws in connected_clients:
        try:
            await ws.send_text(message)
        except Exception:
            dead_clients.add(ws)
    connected_clients.difference_update(dead_clients)


async def _broadcast_state(car_state: Dict[str, dict]):
    await _broadcast(json.dumps({"type": "state", "cars": car_state}))


async def _broadcast_event(event_payload: dict):
    await _broadcast(json.dumps({"type": "event", "event": event_payload}))


# ── FastAPI 라이프사이클 ───────────────────────────────────────────
@app.on_event("startup")
async def startup_event():
    global main_loop
    main_loop = asyncio.get_event_loop()
    mqtt_broker.start()
    print("[APP] 서버 시작")


@app.on_event("shutdown")
async def shutdown_event():
    mqtt_broker.stop()
    print("[APP] 서버 종료")


# ── 엔드포인트 ─────────────────────────────────────────────────────
@app.get("/health")
async def health():
    """서버 상태 및 현재 차량 상태 확인용"""
    return {
        "status": "ok",
        "connected_clients": len(connected_clients),
        "cars": mqtt_broker.car_state,
    }


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    connected_clients.add(websocket)
    print(f"[WS] 클라이언트 접속 (현재 {len(connected_clients)}명)")

    # 접속 직후 현재 상태 스냅샷 바로 전송
    await websocket.send_text(json.dumps({"type": "state", "cars": mqtt_broker.car_state}))

    try:
        while True:
            data = await websocket.receive_text()
            try:
                request = json.loads(data)
            except json.JSONDecodeError:
                print(f"[WS] JSON 파싱 실패: {data}")
                continue
            command = request.get("command")
            target = request.get("target", "all")
            if command:
                mqtt_broker.publish_command(command, target)
    except WebSocketDisconnect:
        connected_clients.discard(websocket)
        print(f"[WS] 클라이언트 종료 (현재 {len(connected_clients)}명)")