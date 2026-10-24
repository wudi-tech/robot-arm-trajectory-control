from pathlib import Path


BASE_DIR = Path(__file__).resolve().parent


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def read_text(name):
    return (BASE_DIR / name).read_text(encoding="utf-8")


def main():
    export_text = read_text("export_pybullet_csv_3dof.m")
    for column in ("dq1_d", "dq2_d", "dq3_d", "ddq1_d", "ddq2_d", "ddq3_d"):
        require(column in export_text, f"CSV export must include {column}")

    urdf_text = read_text("three_dof_spatial_arm.urdf")
    for link_name in ("base", "yaw_hub", "upper_arm", "forearm", "end_effector"):
        require(f'name="{link_name}"' in urdf_text, f"URDF must include {link_name}")
    require(urdf_text.count("<dynamics ") >= 3, "URDF revolute joints must define dynamics damping/friction")

    common_path = BASE_DIR / "pybullet_gravity_tracking.py"
    require(common_path.exists(), "pybullet_gravity_tracking.py must exist")
    common_text = common_path.read_text(encoding="utf-8")
    for token in (
        "setGravity(0, 0, -9.81)",
        "getJointState",
        "calculateInverseDynamics",
        "TORQUE_CONTROL",
        "PID_GAINS",
        "validate_fk_alignment",
    ):
        require(token in common_text, f"Gravity tracking implementation must contain {token}")

    joint_script = read_text("PyBullet_3DOF_sim.py")
    require("quintic_3dof_pybullet.csv" in joint_script, "Joint-space script must read quintic CSV")
    require("pybullet_3dof_simulation.mp4" in joint_script, "Joint-space script must save the joint-space MP4")
    require("run_gravity_tracking" in joint_script, "Joint-space script must use gravity tracking")

    cart_script = read_text("PyBullet_3DOF_cartesian_sim.py")
    require("cartesian_3dof_pybullet.csv" in cart_script, "Cartesian script must read Cartesian CSV")
    require("pybullet_3dof_cartesian_simulation.mp4" in cart_script, "Cartesian script must save the Cartesian MP4")
    require("run_gravity_tracking" in cart_script, "Cartesian script must use gravity tracking")

    print("Gravity PyBullet checks passed.")


if __name__ == "__main__":
    main()
