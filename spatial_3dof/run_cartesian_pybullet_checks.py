from pathlib import Path


BASE_DIR = Path(__file__).resolve().parent


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    main_m = BASE_DIR / "main_advanced_3dof.m"
    cartesian_script = BASE_DIR / "PyBullet_3DOF_cartesian_sim.py"

    main_text = main_m.read_text(encoding="utf-8")
    require(
        "cartesian_3dof_pybullet.csv" in main_text,
        "main_advanced_3dof.m must export cartesian_3dof_pybullet.csv",
    )

    require(cartesian_script.exists(), "PyBullet_3DOF_cartesian_sim.py must exist")
    script_text = cartesian_script.read_text(encoding="utf-8")
    require(
        "cartesian_3dof_pybullet.csv" in script_text,
        "Cartesian PyBullet script must read cartesian_3dof_pybullet.csv",
    )
    require(
        "pybullet_3dof_cartesian_simulation.mp4" in script_text,
        "Cartesian PyBullet script must save pybullet_3dof_cartesian_simulation.mp4",
    )
    require(
        "三维笛卡尔空间规划" in script_text
        or "Cartesian" in script_text
        or "cartesian" in script_text,
        "Cartesian PyBullet script should identify the Cartesian-space trajectory",
    )

    print("Cartesian PyBullet checks passed.")


if __name__ == "__main__":
    main()
