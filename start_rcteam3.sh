#!/bin/bash
# RC Team 3 개발 환경 tmux 세션 실행 스크립트
#
# 사용법:
#   chmod +x start_rcteam3.sh
#   ./start_rcteam3.sh
#
# 이 스크립트 하나로 백엔드 서버 + MQTT 트래픽 모니터 + 테스트용 창을
# tmux 창(window) 4개로 한 번에 띄워줍니다.
#
# tmux 기본 조작:
#   Ctrl+b 그다음 0/1/2/3   -> 해당 번호 창으로 이동
#   Ctrl+b 그다음 d         -> 세션에서 빠져나오기 (백그라운드에서 계속 돌아감)
#   tmux attach -t rcteam3  -> 다시 들어가기
#   tmux kill-session -t rcteam3 -> 세션 완전히 종료

SESSION="rcteam3"
REPO_ROOT="$HOME/github/Rc_team3"
BACKEND_DIR="$REPO_ROOT/kwon/Backend_Dash/backend"
BROKER_IP="172.20.10.5"   # Wi-Fi 재연결로 IP 바뀌면 여기 수정 (mqtt_broker.py의 BROKER_ADDRESS와 항상 동일하게 유지)

# 이미 세션이 떠있으면 새로 만들지 않고 그냥 붙기만 함
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "이미 세션이 실행 중입니다. 접속만 합니다."
    tmux attach -t "$SESSION"
    exit 0
fi

# 0번 창: 백엔드 서버 (FastAPI + MQTT 브릿지)
tmux new-session -d -s "$SESSION" -n backend
tmux send-keys -t "$SESSION:backend" "cd '$BACKEND_DIR' && uvicorn main:app --reload --host 0.0.0.0 --port 8000" C-m

# 1번 창: MQTT 트래픽 전체 모니터 (모든 토픽 실시간으로 보기)
tmux new-window -t "$SESSION" -n mqtt-monitor
tmux send-keys -t "$SESSION:mqtt-monitor" "mosquitto_sub -h $BROKER_IP -t 'rcteam3/#' -v" C-m

# 2번 창: 테스트용 (더미 퍼블리셔 등을 여기서 수동 실행)
tmux new-window -t "$SESSION" -n test
tmux send-keys -t "$SESSION:test" "cd '$REPO_ROOT' && echo '테스트할 때 여기서: python3 dummy_publisher.py'" C-m

# 3번 창: 자유롭게 쓸 일반 셸
tmux new-window -t "$SESSION" -n shell
tmux send-keys -t "$SESSION:shell" "cd '$REPO_ROOT'" C-m

# 0번 창(backend)을 보면서 세션 진입
tmux select-window -t "$SESSION:backend"
tmux attach -t "$SESSION"
