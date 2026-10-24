# Running the archive

## Lightweight checks

From the repository root:

```powershell
matlab -batch "cd('planar_2dof'); run_advanced_checks"
matlab -batch "cd('spatial_3dof'); run_3dof_checks"
python spatial_3dof/run_cartesian_pybullet_checks.py
python spatial_3dof/run_gravity_pybullet_checks.py
```

The MATLAB checks plan the existing 9 s scenarios and run numerical PID tracking; they do not generate videos. The two Python checks read source text and assert export/controller wiring. They do not import PyBullet or prove dynamics performance.

## Full MATLAB comparison (optional)

Run `main_advanced` in `planar_2dof/`, or `main_advanced_3dof` in `spatial_3dof/`. Both create graphics and four MP4 files in the working directory. The 3-DOF entry also exports two CSVs and six PNG/FIG plot pairs into a local `matlab_comparison_figures/` directory. These are regenerated outputs; the archival PNGs are under `results/`.

For only the 2-DOF CSV export, run `quintic_pybullet` in `planar_2dof/`. Its export columns are `t,theta1_d,theta2_d,theta1,theta2`; the PyBullet playback reads the actual angles `theta1,theta2`.

## Python simulation (optional)

Use a Python environment for which the dependencies have compatible wheels. Python 3.10/3.11 is a practical starting point, but this archive does not certify a historical version lock. Create/activate your own environment and install `requirements.txt`.

```powershell
python spatial_3dof/PyBullet_3DOF_sim.py
python spatial_3dof/PyBullet_3DOF_cartesian_sim.py
```

The default is PyBullet DIRECT with the tiny renderer, recording MP4 beside each script. `--gui` opens an interactive scene. Useful options include `--video-stride`, `--hold-frames`, `--width`, `--height`, `--fps` and `--max-rows`. A shortened run such as the following only checks startup and a small prefix, not the full path:

```powershell
python spatial_3dof/PyBullet_3DOF_sim.py --max-rows 100 --hold-frames 0 --width 320 --height 180
```

The existing `verify_pybullet_videos.py` is optional **after generating both full default 3-DOF videos**. It expects 511 frames, 1280×720 and 30 FPS, so it will intentionally fail on shortened/custom-resolution outputs. Videos are not bundled.

`python planar_2dof/PyBullet_sim.py` opens the GUI immediately and records a state-playback video. It is not a torque-control test and has no headless CLI option. Do not import this script as a library: its scene and playback code runs at module scope.

## CSV conventions

The 3-DOF CSVs have 9001 samples, time in seconds, angles in radians, angular velocities in rad/s, angular accelerations in rad/s² and position in metres. `q*_d,dq*_d,ddq*_d,x_d,y_d,z_d` are desired values. `q1,q2,q3,x,y,z` are **MATLAB simplified-plant actual values**, not recorded PyBullet actual states. PyBullet performs a new torque simulation from the desired columns.

Simulation playback/recording speed need not equal the 9 s physical trajectory duration because frames are subsampled and final hold frames are added.
