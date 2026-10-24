from argparse import ArgumentParser
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import pybullet as p
import pybullet_data


BASE_DIR = Path(__file__).resolve().parent
URDF_PATH = BASE_DIR / "three_dof_spatial_arm.urdf"
CONTROL_JOINTS = [0, 1, 2]
END_EFFECTOR_LINK = 3
PHYSICS_DT = 0.001
VIDEO_WIDTH = 1280
VIDEO_HEIGHT = 720
VIDEO_FPS = 30
DEFAULT_VIDEO_STRIDE = 20
DEFAULT_HOLD_FRAMES = 60
PID_GAINS = {
    "kp": np.array([140.0, 180.0, 120.0]),
    "ki": np.array([8.0, 10.0, 6.0]),
    "kd": np.array([22.0, 28.0, 18.0]),
    "integral_limit": np.array([0.35, 0.30, 0.30]),
    "torque_limit": np.array([45.0, 70.0, 55.0]),
}


def build_arg_parser(description):
    parser = ArgumentParser(description=description)
    parser.add_argument("--gui", action="store_true", help="Open the interactive PyBullet GUI while recording.")
    parser.add_argument("--video-stride", type=int, default=DEFAULT_VIDEO_STRIDE, help="Write one frame every N CSV rows.")
    parser.add_argument("--hold-frames", type=int, default=DEFAULT_HOLD_FRAMES, help="Extra final video frames.")
    parser.add_argument("--width", type=int, default=VIDEO_WIDTH, help="Video width.")
    parser.add_argument("--height", type=int, default=VIDEO_HEIGHT, help="Video height.")
    parser.add_argument("--fps", type=int, default=VIDEO_FPS, help="Video frame rate.")
    parser.add_argument("--max-rows", type=int, default=0, help="Use only the first N rows for a quick check.")
    return parser


def create_video_writer(path, width, height, fps):
    if path.exists():
        path.unlink()
    writer = cv2.VideoWriter(
        str(path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width, height),
    )
    if not writer.isOpened():
        raise RuntimeError(f"Cannot open video writer for {path}")
    return writer


def camera_matrices(width, height):
    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[0.35, 0.65, 0.78],
        distance=2.8,
        yaw=45,
        pitch=-30,
        roll=0,
        upAxisIndex=2,
    )
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=50,
        aspect=width / height,
        nearVal=0.01,
        farVal=10,
    )
    return view_matrix, projection_matrix


def write_video_frame(writer, width, height, renderer):
    view_matrix, projection_matrix = camera_matrices(width, height)
    _, _, rgb, _, _ = p.getCameraImage(
        width,
        height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=renderer,
    )
    frame = np.reshape(rgb, (height, width, 4))[:, :, :3]
    writer.write(cv2.cvtColor(np.ascontiguousarray(frame), cv2.COLOR_RGB2BGR))


def add_marker(label, position, color):
    visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.045, rgbaColor=color)
    p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual, basePosition=position)
    p.addUserDebugText(label, [position[0], position[1], position[2] + 0.08], [0, 0, 0], 1.2)


def add_axis_box(name, position, half_extents, color):
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
    p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual, basePosition=position)
    p.addUserDebugText(name, position, color[:3], 0.8)


def add_world_axes():
    add_axis_box("X", [0.225, 0.0, 0.35], [0.225, 0.008, 0.008], [1, 0, 0, 1])
    add_axis_box("Y", [0.0, 0.225, 0.35], [0.008, 0.225, 0.008], [0, 0.7, 0, 1])
    add_axis_box("Z", [0.0, 0.0, 0.625], [0.008, 0.008, 0.625], [0, 0, 1, 1])


def setup_scene(use_gui):
    connection_mode = p.GUI if use_gui else p.DIRECT
    p.connect(connection_mode)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setTimeStep(PHYSICS_DT)
    p.setPhysicsEngineParameter(fixedTimeStep=PHYSICS_DT, numSolverIterations=100)
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)

    if use_gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        p.resetDebugVisualizerCamera(
            cameraDistance=2.8,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0.35, 0.65, 0.78],
        )

    p.loadURDF("plane.urdf")
    robot = p.loadURDF(str(URDF_PATH), [0, 0, 0], useFixedBase=True)
    p.setJointMotorControlArray(robot, CONTROL_JOINTS, p.VELOCITY_CONTROL, forces=[0.0, 0.0, 0.0])

    add_marker("A", [0.95, 0.35, 0.78], [0.9, 0.1, 0.1, 1])
    add_marker("B", [0.55, 1.05, 1.05], [0.1, 0.7, 0.2, 1])
    add_marker("C", [-0.65, 0.85, 0.88], [0.2, 0.2, 0.9, 1])
    add_world_axes()
    return robot


def ensure_derivative_columns(data):
    t = data["t"].to_numpy()
    for joint in range(1, 4):
        q_col = f"q{joint}_d"
        dq_col = f"dq{joint}_d"
        ddq_col = f"ddq{joint}_d"
        if dq_col not in data.columns:
            data[dq_col] = np.gradient(data[q_col].to_numpy(), t)
        if ddq_col not in data.columns:
            data[ddq_col] = np.gradient(data[dq_col].to_numpy(), t)
    return data


def reset_robot_to_desired(robot, row):
    for offset, joint in enumerate(CONTROL_JOINTS, start=1):
        p.resetJointState(robot, joint, float(row[f"q{offset}_d"]), 0.0)


def angle_error(target, current):
    return np.arctan2(np.sin(target - current), np.cos(target - current))


def desired_vectors(row):
    q = np.array([row["q1_d"], row["q2_d"], row["q3_d"]], dtype=float)
    dq = np.array([row["dq1_d"], row["dq2_d"], row["dq3_d"]], dtype=float)
    ddq = np.array([row["ddq1_d"], row["ddq2_d"], row["ddq3_d"]], dtype=float)
    xyz = np.array([row["x_d"], row["y_d"], row["z_d"]], dtype=float)
    return q, dq, ddq, xyz


def current_joint_state(robot):
    states = [p.getJointState(robot, joint) for joint in CONTROL_JOINTS]
    q = np.array([state[0] for state in states], dtype=float)
    dq = np.array([state[1] for state in states], dtype=float)
    return q, dq


def current_end_effector_position(robot):
    link_state = p.getLinkState(robot, END_EFFECTOR_LINK, computeForwardKinematics=True)
    return np.array(link_state[4], dtype=float)


def validate_fk_alignment(robot, data):
    max_err = 0.0
    sample = data.iloc[::DEFAULT_VIDEO_STRIDE]
    for _, row in sample.iterrows():
        reset_robot_to_desired(robot, row)
        ee_pos = current_end_effector_position(robot)
        _, _, _, desired_xyz = desired_vectors(row)
        max_err = max(max_err, float(np.linalg.norm(ee_pos - desired_xyz)))
    print(f"PyBullet FK matches MATLAB CSV, max_err = {max_err:.3e}")
    return max_err


def computed_torque_control(robot, row, integral_error):
    q_d, dq_d, ddq_d, _ = desired_vectors(row)
    q, dq = current_joint_state(robot)
    position_error = angle_error(q_d, q)
    velocity_error = dq_d - dq
    integral_error = integral_error + position_error * PHYSICS_DT
    integral_error = np.clip(
        integral_error,
        -PID_GAINS["integral_limit"],
        PID_GAINS["integral_limit"],
    )

    commanded_accel = (
        ddq_d
        + PID_GAINS["kp"] * position_error
        + PID_GAINS["kd"] * velocity_error
        + PID_GAINS["ki"] * integral_error
    )
    torque = np.array(
        p.calculateInverseDynamics(
            robot,
            q.tolist(),
            dq.tolist(),
            commanded_accel.tolist(),
        ),
        dtype=float,
    )
    torque = np.clip(torque, -PID_GAINS["torque_limit"], PID_GAINS["torque_limit"])
    p.setJointMotorControlArray(robot, CONTROL_JOINTS, p.TORQUE_CONTROL, forces=torque.tolist())
    return integral_error, position_error, torque


def run_gravity_tracking(csv_path, video_path, title, description):
    parser = build_arg_parser(description)
    args = parser.parse_args()

    if args.video_stride < 1:
        raise ValueError("--video-stride must be >= 1")
    if args.hold_frames < 0:
        raise ValueError("--hold-frames must be >= 0")
    if args.width < 160 or args.height < 120:
        raise ValueError("--width/--height are too small for video output")
    if not csv_path.exists():
        raise FileNotFoundError(f"Run main_advanced_3dof.m first to generate {csv_path.name}")

    data = ensure_derivative_columns(pd.read_csv(csv_path))
    if args.max_rows > 0:
        data = data.iloc[: args.max_rows].copy()

    robot = setup_scene(args.gui)
    p.addUserDebugText(title, [-0.55, -0.35, 1.55], [0, 0, 0], 1.2)
    fk_max_err = validate_fk_alignment(robot, data)
    reset_robot_to_desired(robot, data.iloc[0])

    renderer = p.ER_BULLET_HARDWARE_OPENGL if args.gui else p.ER_TINY_RENDERER
    video_writer = create_video_writer(video_path, args.width, args.height, args.fps)
    integral_error = np.zeros(3)
    max_joint_error = np.zeros(3)
    max_ee_error = 0.0
    max_torque = 0.0
    written_frames = 0
    prev_actual = None
    prev_desired = None

    try:
        for row_index, row in data.iterrows():
            integral_error, joint_error, torque = computed_torque_control(robot, row, integral_error)
            p.stepSimulation()

            actual = current_end_effector_position(robot)
            _, _, _, desired = desired_vectors(row)
            max_joint_error = np.maximum(max_joint_error, np.abs(joint_error))
            max_ee_error = max(max_ee_error, float(np.linalg.norm(actual - desired)))
            max_torque = max(max_torque, float(np.max(np.abs(torque))))

            if prev_actual is not None and row_index % args.video_stride == 0:
                p.addUserDebugLine(prev_actual.tolist(), actual.tolist(), [1, 0, 0], 2, 0)
                p.addUserDebugLine(prev_desired.tolist(), desired.tolist(), [0, 0, 1], 1, 0)
            prev_actual = actual
            prev_desired = desired

            if row_index % args.video_stride == 0:
                write_video_frame(video_writer, args.width, args.height, renderer)
                written_frames += 1

        for _ in range(args.hold_frames):
            p.stepSimulation()
            write_video_frame(video_writer, args.width, args.height, renderer)
            written_frames += 1
    finally:
        video_writer.release()
        p.disconnect()

    print(f"Gravity enabled: setGravity(0, 0, -9.81)")
    print(f"Video saved to {video_path}")
    print(f"Video frames written: {written_frames}")
    print(f"FK alignment max error: {fk_max_err:.3e} m")
    print(f"Max joint tracking error: {np.max(max_joint_error) * 180 / np.pi:.3f} deg")
    print(f"Max end-effector tracking error: {max_ee_error:.4f} m")
    print(f"Max commanded torque: {max_torque:.3f} N*m")
