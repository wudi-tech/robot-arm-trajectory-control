import pybullet as p
import pybullet_data
import pandas as pd
import time
import math
import cv2
import numpy as np
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
URDF_PATH = BASE_DIR / "two_link_arm.urdf"
CSV_PATH = BASE_DIR / "quintic_pybullet.csv"
VIDEO_PATH = BASE_DIR / "pybullet_2dof_simulation.mp4"
VIDEO_WIDTH = 1280
VIDEO_HEIGHT = 720
VIDEO_FPS = 30


def create_video_writer(path):
    if path.exists():
        path.unlink()
    writer = cv2.VideoWriter(
        str(path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        VIDEO_FPS,
        (VIDEO_WIDTH, VIDEO_HEIGHT),
    )
    if not writer.isOpened():
        raise RuntimeError(f"Cannot open video writer for {path}")
    return writer


def write_video_frame(writer):
    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[0.65, 0.75, 0.28],
        distance=2.35,
        yaw=42,
        pitch=-38,
        roll=0,
        upAxisIndex=2,
    )
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=50,
        aspect=VIDEO_WIDTH / VIDEO_HEIGHT,
        nearVal=0.01,
        farVal=10,
    )
    _, _, rgb, _, _ = p.getCameraImage(
        VIDEO_WIDTH,
        VIDEO_HEIGHT,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL,
    )
    frame = np.reshape(rgb, (VIDEO_HEIGHT, VIDEO_WIDTH, 4))[:, :, :3]
    writer.write(cv2.cvtColor(np.ascontiguousarray(frame), cv2.COLOR_RGB2BGR))

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
p.resetDebugVisualizerCamera(
    cameraDistance=2.35,
    cameraYaw=42,
    cameraPitch=-38,
    cameraTargetPosition=[0.65, 0.75, 0.28],
)

p.loadURDF("plane.urdf")
robot = p.loadURDF(str(URDF_PATH), [0, 0, 0.0], useFixedBase=True)

data = pd.read_csv(CSV_PATH)

L1 = 1.0
L2 = 0.8
ee_height = 0.40
trace_height = 0.58
prev = None

p.addUserDebugText("2-DOF Robot Arm: MATLAB PID Tracking Trajectory", [-0.35, -0.42, 0.72], [0, 0, 0], 1.2)
p.addUserDebugText("A", [1.37, 0.95, trace_height + 0.08], [0, 0, 0], 1.2)
p.addUserDebugText("B", [0.12, 1.42, trace_height + 0.08], [0, 0, 0], 1.2)
p.addUserDebugText("C", [-0.07, 1.51, trace_height + 0.08], [0, 0, 0], 1.2)

p.addUserDebugLine([0, 0, 0.41], [0.4, 0, 0.41], [1, 0, 0], 4, 0)
p.addUserDebugLine([0, 0, 0.41], [0, 0.4, 0.41], [0, 1, 0], 4, 0)
p.addUserDebugLine([0, 0, 0.0], [0, 0, 0.75], [0, 0, 1], 4, 0)

for marker_pos, color in [
    ([1.37, 0.95, trace_height], [0.9, 0.1, 0.1, 1]),
    ([0.12, 1.42, trace_height], [0.1, 0.7, 0.2, 1]),
    ([-0.07, 1.51, trace_height], [0.2, 0.2, 0.9, 1]),
]:
    marker_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.035, rgbaColor=color)
    p.createMultiBody(baseMass=0, baseVisualShapeIndex=marker_visual, basePosition=marker_pos)

video_writer = create_video_writer(VIDEO_PATH)

for _, row in data.iloc[::20].iterrows():
    theta1 = row["theta1"]
    theta2 = row["theta2"]

    p.resetJointState(robot, 0, theta1)
    p.resetJointState(robot, 1, theta2)

    x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
    y = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)
    pos = [x, y, trace_height]

    if prev is not None:
        p.addUserDebugLine(prev, pos, [1, 0, 0], 2, 0)

    prev = pos

    p.stepSimulation()
    write_video_frame(video_writer)
    time.sleep(1 / 60)

for _ in range(VIDEO_FPS * 2):
    p.stepSimulation()
    write_video_frame(video_writer)
    time.sleep(1 / 60)

video_writer.release()
print(f"PyBullet 2-DOF video saved to {VIDEO_PATH}")
p.disconnect()
