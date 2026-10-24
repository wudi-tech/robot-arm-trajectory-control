# 机械臂轨迹规划与控制仿真

本归档将二自由度平面机械臂与三自由度空间机械臂整理为同一进阶项目：从平面运动学、轨迹规划与 PID 对比，扩展到 yaw-pitch-pitch 空间运动学、三维末端路径与 PyBullet 重力力矩控制。

## 项目与贡献范围

仓库所有者已明确确认：本人的运动学建模、轨迹规划与控制仿真实现为独立开发。课程提交材料也记录了分组报告、演示文稿及汇报工作；这些其他成员的写作与展示贡献不归为本人独立完成。个人技术职责以本人确认优先，技术指标以实际文件与验证为准。三次/五次多项式、梯形速度、几何逆运动学与 PID 是标准方法，不主张算法原创。

本项目全部证据均来自仿真。三自由度结构只控制末端位置 `(x,y,z)`，不能独立控制完整六维位姿。A/B/C 仅模拟等待、取料和放料路径；没有夹爪、物体接触、抓取或释放验证。

## 目录与入口

| 目录 | 内容 | 主要入口 |
|---|---|---|
| `planar_2dof/` | 平面二连杆、四种规划与简化 PID；保留早期单文件基线 | `main_advanced.m`、`run_advanced_checks.m` |
| `spatial_3dof/` | 空间运动学、四种规划、MATLAB 导出、URDF 与重力控制 | `main_advanced_3dof.m`、`run_3dof_checks.m`、`PyBullet_3DOF_sim.py` |
| `results/spatial_3dof/` | 六张 MATLAB 小图、已有模型截图、重力轨迹图 | [结果索引](RESULTS.md) |

三个 CSV 与读取它们的代码放在同目录，保留原入口的相对路径。所有选入源文件的来源文件名、SHA-256 和大小记录在根目录 `SOURCE_MANIFEST.csv`。

## 必须分开的两套模型

MATLAB 使用独立关节简化动力学 `J*qdd+B*qd=u`，仅含等效惯量和阻尼，不含重力、关节耦合、科氏/离心项。PID 输出力矩后，数值积分得到实际关节状态。

二自由度 PyBullet 脚本读取 MATLAB 实际关节角，并逐帧 `resetJointState`，属于运动学回放；不能作为重力动力学跟踪证据。

三自由度 PyBullet 使用 URDF 多刚体模型，设置 `g=-9.81 m/s²`，读取期望角度/角速度/角加速度，结合当前关节状态生成指令加速度：

```text
qdd_cmd = qdd_d + Kp*(q_d-q) + Kd*(qd_d-qd) + Ki*积分(q_d-q)
tau = inverseDynamics(q, qd, qdd_cmd)
```

计算所得力矩再做限幅，通过 `TORQUE_CONTROL` 驱动。控制器与 MATLAB 使用不同增益，URDF 质量惯量也不等同于 MATLAB 的等效关节惯量，因此不能把两套误差或力矩直接视为统一对照实验。控制器力矩上限与 URDF effort 字段不一致，当前并未验证真实执行器可行性。

## 轨迹与既有结论

每段 3 s，共 A→B→C→A 三段，步长 0.001 s。三次只约束端点速度，段间加速度可以不连续；五次约束端点速度和加速度为零，但不保证加加速度连续；梯形在阶段切换处有加速度阶跃。笛卡尔空间采用直线位置插值与五次时间律，再通过 IK 求关节轨迹，导数使用数值梯度。

修订报告的三自由度末端 RMSE 为：三次 1.834 mm、五次 2.026 mm、梯形 1.834 mm、笛卡尔 1.853 mm。五次的优势是端点约束与运动平滑性，不能写成当前参数下精度最好。角速度图比较的是规划角速度，不是实际角速度跟踪。

修订报告记录的 PyBullet 最大末端误差约为关节空间 0.7 mm、笛卡尔空间 1.0 mm，采样 FK 对齐误差约 `1e-7 m`。这些是已有报告中的舍入结果，本次不重跑大型仿真。CSV 内的实际状态来自 MATLAB，不能用来重新计算 PyBullet 的实际跟踪指标。更完整的定量表与证据边界见 [RESULTS.md](RESULTS.md)。

## 运行与验证

从各自目录运行 MATLAB，避免同名函数受路径影响。已有检查脚本只做轻量数值检查，不输出视频：

```matlab
% 在 planar_2dof 目录
run_advanced_checks
% 在 spatial_3dof 目录
run_3dof_checks
```

完整主程序会生成图像、CSV 和多个视频；Python 入口、依赖和可选短运行参数见 [RUNNING.md](RUNNING.md)。`requirements.txt` 仅列依赖，未假称恢复历史环境。两个 Python `run_*pybullet_checks.py` 为静态接线检查，不等同于 PyBullet 仿真通过。本次实际检查记录见 [VALIDATION.md](VALIDATION.md)。

## 公开选择与局限

源代码、URDF、CSV 和精选已有图片保留原始字节；整份含名单/学号报告、各版 PPT、重复压缩包、视频、`.fig`、缓存、编辑器配置与原聊天不公开。原工程不修改，不把私人路径写入公开文档。

当前缺少真实机械臂验证、统一动力学参数辨识、负载/扰动对照、约束规划、避障与抓取过程。PyBullet 历史结果也未附逐步状态日志。版权、来源与团队边界见 [PROVENANCE.md](PROVENANCE.md) 与 [THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md)。
