"""
test_command_client.py — 명령 경로(start/stop/emergency_stop) 테스트용

대시보드가 아직 없어도, 이 스크립트가 대시보드 대신
백엔드의 WebSocket(/ws)에 붙어서 명령을 보내본다.

백엔드가 명령을 잘 받으면 MQTT의 command 토픽으로 재발행하는데,
이건 tmux 1번(mqtt-monitor) 창에서 눈으로 확인하면 된다.

사용법:
    python3 test_command_client.py
"""

import asyncio
import json

import websockets

# 백엔드가 같은 컴퓨터(WSL)에서 돌고 있으면 localhost로 충분함
BACKEND_WS_URL = "ws://localhost:8000/ws"

# 순서대로 테스트해볼 명령들
TEST_COMMANDS = [
    {"command": "start", "target": "all"},
    {"command": "stop", "target": "A"},
    {"command": "emergency_stop", "target": "all"},
]


async def main():
    print(f"백엔드에 연결 시도: {BACKEND_WS_URL}")
    async with websockets.connect(BACKEND_WS_URL) as ws:
        print("연결 성공!")

        # 접속 직후 서버가 보내주는 현재 상태 스냅샷 한 번 받기
        first_message = await ws.recv()
        print(f"[서버로부터 초기 상태 수신] {first_message}")

        for cmd in TEST_COMMANDS:
            print(f"\n>>> 명령 전송: {cmd}")
            await ws.send(json.dumps(cmd))
            await asyncio.sleep(2)  # tmux mqtt-monitor 창에서 확인할 시간

        print("\n테스트 명령 3개 모두 전송 완료.")
        print("tmux 1번(mqtt-monitor) 창에서 rcteam3/autocar/.../command 토픽으로")
        print("이 3개 명령이 순서대로 찍혔는지 확인하세요.")


if __name__ == "__main__":
    asyncio.run(main())
