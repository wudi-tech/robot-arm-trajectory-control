from pathlib import Path

import cv2


BASE_DIR = Path(__file__).resolve().parent
VIDEO_FILES = [
    BASE_DIR / "pybullet_3dof_simulation.mp4",
    BASE_DIR / "pybullet_3dof_cartesian_simulation.mp4",
]


def main():
    for video_file in VIDEO_FILES:
        cap = cv2.VideoCapture(str(video_file))
        opened = cap.isOpened()
        frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        ok, frame = cap.read()
        frame_mean = None if frame is None else round(float(frame.mean()), 2)
        frame_std = None if frame is None else round(float(frame.std()), 2)
        cap.release()

        if not opened or not ok:
            raise RuntimeError(f"Cannot read {video_file.name}")
        if frames != 511 or round(fps) != 30 or width != 1280 or height != 720:
            raise RuntimeError(
                f"{video_file.name} expected 511 frames at 1280x720/30 FPS, "
                f"got {frames} frames at {width}x{height}/{fps} FPS"
            )

        print(
            f"{video_file.name}: opened={opened}, frames={frames}, fps={fps}, "
            f"size={width}x{height}, first_frame={ok}, mean={frame_mean}, std={frame_std}"
        )


if __name__ == "__main__":
    main()
