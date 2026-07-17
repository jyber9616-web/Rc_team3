# 1. pip 패키지 매니저를 최신 버전으로 업데이트합니다.
# !pip install --upgrade pip

# 2. 그 다음 다시 설치를 시도합니다.
# !pip install inference-sdk

# 1. OpenCV를 제외한 필수 가벼운 패키지들만 먼저 설치합니다.
# !pip install requests requests-toolbelt tqdm wget certifi urllib3 chardet

# 2. 의존성 검사를 무시하고 roboflow만 강제로 안전하게 설치합니다.
# !pip install roboflow==0.2.32 --no-deps

# !pip install -U python-dotenv

import sys
import time
import cv2
import numpy as np
import os
import requests
import tensorflow as tf
from pop import Pilot 

# 1. GPU 메모리 동적 할당
gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
    try:
        tf.config.experimental.set_memory_growth(gpus[0], True)
        print("✅ GPU 메모리 동적 할당 완료")
    except RuntimeError as e:
        print(e)

# 2. 통신 및 모델 정보 설정
MY_API_KEY = "5qqbBd2raZ4VbubgCOgG" 
PROJECT_NAME = "autocar-q0bqc"
MODEL_VERSION = 1

TARGET_URL = f"https://detect.roboflow.com/{PROJECT_NAME}/{MODEL_VERSION}"
PARAMS = {
    "api_key": MY_API_KEY,
    "confidence": "40",
    "overlap": "30",
    "format": "json"
}
print("✅ Roboflow 다이렉트 통신 셋팅 완료")

# 3. 하드웨어 초기화
try:
    if 'cam' in locals(): cam.stop(); del cam
    if 'ac' in locals(): ac.stop()
except Exception:
    pass

cam = Pilot.Camera(width=320, height=320) 
ac = Pilot.AutoCar() 

print("🧹 카메라 버퍼 비우는 중...")
for _ in range(10):
    if hasattr(cam, 'read'): cam.read()
    elif callable(cam): cam()
    time.sleep(0.05)

STOP_SIZE_RATE = 0.35
print("🚗 AutoCar 주행을 시작합니다! (화면창 제외 안정 버전)")

# 4. 메인 주행 루프
try:
    while True:
        raw_img = cam.read() if hasattr(cam, 'read') else cam()
        
        if raw_img is not None:
            if hasattr(raw_img, 'value') and not isinstance(raw_img, np.ndarray):
                img_bytes = raw_img.value
                if isinstance(img_bytes, bytes) and len(img_bytes) > 0:
                    nparr = np.frombuffer(img_bytes, dtype=np.uint8)
                    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                else:
                    continue
            else:
                frame = raw_img
                
            if frame is None or not isinstance(frame, np.ndarray):
                continue

            # 🚨 충돌 유발 원인인 cv2.imshow / waitKey를 제거했습니다.
            cv2.imwrite("temp_frame.jpg", frame)
            
            try:
                with open("temp_frame.jpg", "rb") as f:
                    response = requests.post(TARGET_URL, params=PARAMS, files={"file": f}, timeout=3)
                
                if response.status_code == 200:
                    predictions = response.json().get('predictions', [])
                else:
                    predictions = []
            except Exception:
                predictions = []
            
            # 주행 제어 로직
            if len(predictions) > 0:
                target = predictions[0]
                
                # 조향 제어 (Center X 기반)
                center_x = target['x']          
                offset_x = (center_x - 160) / 160  
                steer = np.clip(offset_x * 4.0, -1.0, 1.0)
                ac.steering = steer
                
                # 속도 제어 (Object Size 기반)
                box_area = target['width'] * target['height']
                size_rate = box_area / (320 * 320)
                
                if size_rate < STOP_SIZE_RATE:
                    ac.forward(50)
                    print(f"🏃 [인식] 전진 중 (크기: {size_rate:.2f})       ", end='\r')
                else:
                    ac.backward(50)
                    print(f"🔙 [근접] 후진 중 (크기: {size_rate:.2f})       ", end='\r')
            else:
                ac.stop()
                print("🔍 오토카 탐색 중...                            ", end='\r')
        else:
            ac.stop()
            
        time.sleep(0.03) # 통신 주기 미세 조정

except (KeyboardInterrupt, SystemExit):
    print("\n🛑 사용자에 의해 주행이 중단되었습니다.")

finally:
    print("\n🧹 자원 초기화 중...")
    ac.stop()
    ac.steering = 0
    cam.stop()
    if os.path.exists("temp_frame.jpg"):
        os.remove("temp_frame.jpg")
    print("✨ 초기화 완료!")