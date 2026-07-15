import json
import os
from pathlib import Path

from inference_sdk import InferenceHTTPClient


BASE_DIR = Path(__file__).resolve().parent

# split_video.py로 생성한 이미지 폴더
IMAGE_DIR = BASE_DIR / "video_to_frame/frames" / "input_video"

MODEL_ID = "rc_lane/1"


def find_test_image() -> Path:
    """프레임 폴더에서 첫 번째 JPG 이미지를 찾습니다."""

    if not IMAGE_DIR.exists():
        raise FileNotFoundError(
            f"이미지 폴더를 찾을 수 없습니다:\n{IMAGE_DIR}"
        )

    image_paths = sorted(IMAGE_DIR.glob("*.jpg"))

    if not image_paths:
        raise FileNotFoundError(
            f"JPG 이미지가 없습니다:\n{IMAGE_DIR}"
        )

    return image_paths[0]


def main() -> None:
    api_key = os.environ.get("ROBOFLOW_API_KEY")

    if not api_key:
        raise RuntimeError(
            "ROBOFLOW_API_KEY 환경변수가 설정되지 않았습니다.\n"
            "터미널에서 다음 형식으로 설정하십시오:\n"
            "export ROBOFLOW_API_KEY='API_KEY'"
        )

    image_path = find_test_image()

    client = InferenceHTTPClient(
        api_url="https://serverless.roboflow.com",
        api_key=api_key,
    )

    print(f"사용 이미지: {image_path}")
    print(f"사용 모델: {MODEL_ID}")
    print("추론 요청 중...")

    result = client.infer(
        str(image_path),
        model_id=MODEL_ID,
    )

    print("\n추론 결과:")
    print(
        json.dumps(
            result,
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()