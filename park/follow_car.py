# 차량 객체추종

# 1. 🚨 무조건 '가장 먼저' 실행
import sys
import os

try:
    import ctypes
    ctypes.CDLL('/usr/lib/aarch64-linux-gnu/libgomp.so.1', mode=ctypes.RTLD_GLOBAL)
    print("✅ 시스템 TLS 메모리 우회 블록 주입 성공!")
except Exception as e:
    print(f"⚠️ 주입 시도 중 알림: {e}")

# 2. 모듈 로드
import tensorflow as tf
import time

gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
    try:
        tf.config.experimental.set_memory_growth(gpus[0], True)
        print("✅ GPU 메모리 동적 할당 완료")
    except RuntimeError as e:
        print(e)

# 3. 라이브러리 불러오기
from pop import Pilot 

try:
    if 'cam' in locals(): cam.stop(); del cam
    if 'ac' in locals(): ac.stop()
except Exception:
    pass

cam = Pilot.Camera(width=320, height=320) 
ac = Pilot.AutoCar() 
OF = Pilot.Object_Follow(cam) 
OF.load_model() 

# 💡 [핵심 보완] 옛날 이미지 찌꺼기 털어내기
# 카메라를 켜자마자 바로 화면을 띄우지 않고, 버퍼에 고여있던 옛날 프레임을 10장쯤 흘려보냅니다.
print("🧹 카메라 버퍼의 예전 이미지 찌꺼기를 청소하는 중...")
for _ in range(10):
    try:
        if hasattr(cam, 'read_image'): cam.read_image()
        elif hasattr(cam, 'np_array'): cam.np_array()
    except Exception:
        pass
    time.sleep(0.05)

# 📺 이제 깨끗해진 채널로 뷰어를 켭니다.
OF.show()

STOP_SIZE_RATE = 0.10
print("🚗 오토카 객체 추종 주행을 시작합니다!")

try:
    while True:
        v = OF.detect(index='car') 
        
        if v is not None and isinstance(v, dict) and 'box' in v:
            steer = v['x'] * 4
            if steer > 1: steer = 1
            elif steer < -1: steer = -1
            ac.steering = steer
            
            if v['size_rate'] < STOP_SIZE_RATE:
                ac.forward(50)
                print(f"🏃 목표 차량 추적 중 (전진)... 크기 비율: {v['size_rate']:.2f}  ", end='\r', flush=True)
            else:
                ac.backward(50)
                print(f"🔙 너무 가깝습니다! 후진 중... 크기 비율: {v['size_rate']:.2f}  ", end='\r', flush=True)
        else:
            ac.stop()
            print("🔍 차량을 찾는 중입니다...                           ", end='\r', flush=True)
            
        time.sleep(0.05)

except (KeyboardInterrupt, SystemExit, Exception) as e:
    print(f"\n🛑 주행이 중단되었습니다. (원인: {type(e).__name__} - {e})")

finally:
    print("🧹 오토카 자원 및 카메라 뷰를 초기화하는 중...")
    try:
        ac.stop()
        ac.steering = 0
        cam.stop()
        del cam
        print("✨ 초기화 완료!")
    except NameError:
        pass