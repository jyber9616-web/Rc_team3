from pathlib import Path

import cv2


# ==================================================
# 사용자 설정
# ==================================================

# 분할할 영상 파일
BASE_DIR = Path(__file__).resolve().parent
VIDEO_PATH = BASE_DIR / "input_video.mp4"
OUTPUT_ROOT = BASE_DIR / "frames"

# 1이면 모든 프레임 저장
# 2이면 2프레임마다 1장 저장
# 5이면 5프레임마다 1장 저장
SAVE_EVERY_N_FRAMES = 1

# JPG 품질: 0~100
JPEG_QUALITY = 95


def split_video_into_frames() -> None:
    """영상 파일을 읽어서 개별 JPG 프레임으로 저장합니다."""

    if not VIDEO_PATH.exists():
        raise FileNotFoundError(
            f"영상 파일을 찾을 수 없습니다: {VIDEO_PATH.resolve()}"
        )

    # 영상 이름별로 별도의 폴더 생성
    output_dir = OUTPUT_ROOT / VIDEO_PATH.stem
    output_dir.mkdir(parents=True, exist_ok=True)

    capture = cv2.VideoCapture(str(VIDEO_PATH))

    if not capture.isOpened():
        raise RuntimeError(
            f"영상을 열 수 없습니다: {VIDEO_PATH.resolve()}"
        )

    fps = capture.get(cv2.CAP_PROP_FPS)
    total_frames = int(
        capture.get(cv2.CAP_PROP_FRAME_COUNT)
    )
    frame_width = int(
        capture.get(cv2.CAP_PROP_FRAME_WIDTH)
    )
    frame_height = int(
        capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
    )

    print("=" * 50)
    print(f"영상 파일: {VIDEO_PATH.resolve()}")
    print(f"영상 FPS: {fps:.2f}")
    print(f"전체 프레임 수: {total_frames}")
    print(f"해상도: {frame_width} × {frame_height}")
    print(f"저장 간격: {SAVE_EVERY_N_FRAMES}프레임마다 1장")
    print(f"저장 위치: {output_dir.resolve()}")
    print("=" * 50)

    frame_index = 0
    saved_count = 0

    while True:
        success, frame = capture.read()

        # 더 이상 읽을 프레임이 없으면 종료
        if not success:
            break

        # SAVE_EVERY_N_FRAMES가 1이면 모든 프레임 저장
        if frame_index % SAVE_EVERY_N_FRAMES == 0:
            filename = (
                f"{VIDEO_PATH.stem}_"
                f"frame_{frame_index:08d}.jpg"
            )

            output_path = output_dir / filename

            saved = cv2.imwrite(
                str(output_path),
                frame,
                [
                    cv2.IMWRITE_JPEG_QUALITY,
                    JPEG_QUALITY,
                ],
            )

            if not saved:
                print(f"저장 실패: {output_path}")
            else:
                saved_count += 1

        frame_index += 1

        # 진행 상황 출력
        if frame_index % 500 == 0:
            print(
                f"처리 중: {frame_index}/{total_frames}프레임, "
                f"저장: {saved_count}장"
            )

    capture.release()

    print("\n프레임 분할 완료")
    print(f"읽은 프레임 수: {frame_index}")
    print(f"저장된 사진 수: {saved_count}")
    print(f"결과 폴더: {output_dir.resolve()}")


if __name__ == "__main__":
    split_video_into_frames()