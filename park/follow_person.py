#사람 객체추종

from pop import Pilot 
import time

# 💡 이전 실행에서 미처 꺼지지 않은 자원이 있다면 선행 청소
try:
    if 'cam' in locals():
        cam.stop()
        del cam
    if 'ac' in locals():
        ac.stop()
except Exception:
    pass

# 하드웨어 및 인공지능 모델 할당
cam = Pilot.Camera(width=320, height=320) 
ac = Pilot.AutoCar() 
OF = Pilot.Object_Follow(cam) 
OF.load_model() 

# 주피터 화면에 실시간 영상 창을 띄웁니다.
cam.show()

# 거리 조절 기준 설정 (0.10: 화면의 10%를 차지할 때 기준)
STOP_SIZE_RATE = 0.10

print("🚶 사람이 나타나면 거리를 유지하며 (멀면 전진, 가까우면 후진) 따라가는 주행을 시작합니다.")

try:
    while True:
        v = OF.detect(index='person')
        
        # 사람이 화면에 감지되었을 때
        if v is not None:
            # 1. 방향(조향) 계산 및 안전 범위 제안 (-1.0 ~ 1.0)
            # 사람이 있는 방향을 바라보도록 바퀴를 정렬합니다.
            steer = v['x'] * 4
            if steer > 1:
                steer = 1
            elif steer < -1:
                steer = -1
                
            ac.steering = steer
            
            # 2. [거리 및 주행 제어 구역]
            if v['size_rate'] < STOP_SIZE_RATE:
                # 🏃 아직 멀리 있다면 (사람 크기가 기준보다 작으면) 전진하며 추적
                ac.forward(50)
                print(f"🏃 사람 추적 중 (전진)... 크기 비율: {v['size_rate']:.2f}", end='\r')
            else:
                # 🔙 너무 가까워지면 (사람 크기가 기준보다 크면) 왔던 길 방향을 유지하며 후진!
                # 속도 50으로 안전하게 뒤로 물러납니다.
                ac.backward(50)
                print(f"🔙 너무 가깝습니다! 후진 중... 크기 비율: {v['size_rate']:.2f}", end='\r')
                
        # 사람이 화면에서 완전히 사라졌을 때
        else:
            # 안전을 위해 자리에 멈춥니다.
            ac.stop()
            
        time.sleep(0.05) # 연산 과부하 및 서버 커넥션 에러 방지

# 주피터 정지(■) 버튼을 누르면 이 구역이 실행됩니다.
except (KeyboardInterrupt, SystemExit, Exception) as e:
    print(f"\n🛑 주행이 중단되었습니다. (원인: {type(e).__name__})")

finally:
    # 🧼 메모리와 하드웨어 자원을 자동으로 깔끔하게 청소하는 구역
    print("🧹 오토카 자원 및 카메라 뷰를 초기화하는 중...")
    try:
        ac.stop()       # 모터 정지
        ac.steering = 0 # 바퀴 정렬
        cam.stop()      # 📸 카메라 하드웨어 및 실시간 뷰어 종료
        del cam         # 메모리에서 카메라 객체 완전 삭제
        print("✨ 초기화 완료! 다음 실행 시 멈춤 없이 바로 재실행 가능합니다.")
    except NameError:
        pass