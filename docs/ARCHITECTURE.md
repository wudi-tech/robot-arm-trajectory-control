# Architecture and mathematical boundary

## Data flow

```text
Parameters + A/B/C waypoints
    -> geometric IK (where Cartesian waypoints are used)
    -> cubic / quintic / trapezoidal / Cartesian planner
    -> desired q, dq, ddq
    -> MATLAB PID + independent-joint plant
    -> metrics + plots + CSV
    -> [3-DOF] desired CSV columns + URDF
    -> state feedback + inverse dynamics + torque control in PyBullet
```

The 2-DOF PyBullet branch instead reads the MATLAB actual-angle CSV columns and forces the displayed joint states. It is a visualization branch.

## Geometry

The planar arm has lengths 1.0 m and 0.8 m. Its elbow branch is selected by the sign of `acos(c2)`; `atan2` determines the shoulder angle. The spatial arm uses base height H=0.35 m, upper arm L1=0.85 m and forearm L2=0.65 m:

```text
r = L1*cos(q2) + L2*cos(q2+q3)
x = r*cos(q1)
y = r*sin(q1)
z = H + L1*sin(q2) + L2*sin(q2+q3)
q1 = atan2(y,x)
```

The spatial IK also uses `atan2` for the shoulder, clips only roundoff-level cosine-domain excess and rejects unreachable targets. Positive elbow branch and `unwrap` are used in the existing scenarios. This is not a general singularity-handling or joint-limit-aware solver.

## Time laws

With `s=t/T`:

- Cubic: `h=3*s^2-2*s^3`, zero endpoint velocity; segment accelerations need not match.
- Quintic: `h=10*s^3-15*s^4+6*s^5`, zero endpoint velocity and acceleration; jerk is not constrained to be continuous.
- Trapezoidal velocity: acceleration / constant velocity / deceleration; acceleration ratio is 0.25 of a segment. Acceleration steps remain.
- Cartesian: `p=p_start+h*(p_end-p_start)` using quintic timing, followed by IK. `dq` and `ddq` use numerical gradients.

All existing runs use 3 s per segment, 1 ms numerical steps and A→B→C→A (9 s). A small numerical step does not itself guarantee accuracy or safe motion.

## MATLAB plant

`simulate_pid_tracking*.m` computes error, an accumulated integral and a finite-difference derivative. PID torque drives `qdd=(u-B*dq)/J`, then velocity and position are integrated. No saturation or anti-windup is implemented here. Parameters and gains live in the respective `init_params*.m` files. The initial actual position matches the initial desired position and initial actual velocity is zero.

## 3-DOF PyBullet plant

`pybullet_gravity_tracking.py` sets a 1 ms physics step and 100 solver iterations, loads five links and three revolute joints plus an end-effector fixed joint, disables default velocity motors and enables gravity. The controller uses measured angle/velocity, angle-wrapped position error, integral clipping and acceleration feedforward. Inverse dynamics computes the multibody torque before command clipping.

The gains are `Kp=[140,180,120]`, `Ki=[8,10,6]`, `Kd=[22,28,18]`; integral limits `[0.35,0.30,0.30]`, command limits `[45,70,55]` N·m. These differ from the MATLAB PID gains and equivalent joint inertia/damping. Command limits also differ from the URDF effort fields `[30,30,25]`; the existing controller does not derive its clipping from those fields. No actuator-feasibility claim is made.

`validate_fk_alignment` resets a sampled set of desired poses solely to compare geometry, then the robot is reset once to the initial pose before torque tracking. The reported PyBullet joint error is sampled before a physics step; the end-effector error is measured after the step against that row's desired position. `--hold-frames` advances the final scene without issuing fresh feedback commands, and those hold frames are excluded from the printed maxima. These details limit interpretation of historical tracking metrics.
