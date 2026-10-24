# Archive validation

Checked on 2026-10-25. These checks validate the selected archive; historical performance values remain labeled as such.

- MATLAB R2022a: ran `run_advanced_checks` from `planar_2dof`, then `run_3dof_checks` from `spatial_3dof` with function state cleared. Both printed their pass messages; the batch process exited with code 0. MATLAB also emitted a Java connector shutdown exception after the checks; it did not fail the batch exit.
- Python 3.14: ran `run_cartesian_pybullet_checks.py` and `run_gravity_pybullet_checks.py`. Both passed. These scripts inspect source wiring; they do not execute a PyBullet dynamics simulation.
- The archive audit compares copied source hashes, parses Python and URDF, checks CSV time grids and forward-kinematics consistency, opens the selected images, and checks local documentation links and private-path patterns. Its private report is kept outside the repository.
- The eight selected historical figures were visually reviewed for personal information. The gravity comparison image has missing Chinese font glyphs in its original labels; it is preserved as historical evidence.

No full animation/video generation, new gravity simulation, hardware operation or new performance study was performed during publication preparation. Source programs and existing result files retain their original bytes.

The original `planar_2dof/main_baseline.m` includes trailing-whitespace lines. They were inspected and retained to preserve the original source hash; no other whitespace errors were found.
