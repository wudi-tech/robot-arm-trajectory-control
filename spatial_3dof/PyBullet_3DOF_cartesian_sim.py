from pathlib import Path

from pybullet_gravity_tracking import run_gravity_tracking


BASE_DIR = Path(__file__).resolve().parent
CSV_PATH = BASE_DIR / "cartesian_3dof_pybullet.csv"
VIDEO_PATH = BASE_DIR / "pybullet_3dof_cartesian_simulation.mp4"


if __name__ == "__main__":
    run_gravity_tracking(
        csv_path=CSV_PATH,
        video_path=VIDEO_PATH,
        title="3-DOF Spatial Robot Arm: Cartesian-space trajectory",
        description="Render the gravity-enabled Cartesian-space PyBullet simulation.",
    )
