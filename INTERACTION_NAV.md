# 双房间交互导航

第一版使用已知场景几何、机器人真实位姿和自动门接口。机器人自主接近门、
停稳、请求打开、检查通行空间、穿门并驶向绿色目标。它不是视觉识别门或机械臂开门系统。

## 运行

### 先生成单张交互路径图（不运行机器人）

```bash
python plot_interaction_path.py
# 仅保存，不弹出窗口
python plot_interaction_path.py --no-show
# 检查通道受阻的规划结果
python plot_interaction_path.py --scenario blocked --no-show
```

该入口仅需要 NumPy、PyYAML 和 Matplotlib，不加载 iGibson/PyBullet。
复用任务的配置、地图、A* 和路径简化，在一张俯视图中显示蓝色第一段导航、
橙色停车/开门/通行确认动作、绿色第二段导航（穿门并到达目标）。
关闭门板用虚线，计划打开位置用实线，扇形表示开门扫掠区域。
灰色膨胀范围按计划开门角度绘制，绿色路径必须在实际角度和通道检查通过后才能执行。

默认保存 `outputs/interaction_planning/interaction_plan_closed.png` 和同名 JSON，
其他场景使用各自的后缀；同场景重复生成会覆盖。JSON 包含分段航点、路径长度、
交互位置、动作顺序和执行条件。可通过 `--output-dir` 指定其他目录。
图中文字采用英文，以兼容未安装中文字体的仿真环境。

这是基于精确配置点的条件规划，不是实际轨迹；执行时系统会从测得的位姿重新规划。
若通道受阻，不绘制第二段可执行路径，并以红字说明原因；`jammed` 场景保留
假设开门后的几何路径，但明确标注开门会失败、绿色路线仅为假设。

### 运行自主任务

在能运行 `interaction_scene.py` 的 iGibson 2.x 环境中，从项目目录执行：

```bash
python run_interaction_nav.py
python run_interaction_nav.py --scenario open
python run_interaction_nav.py --scenario jammed
python run_interaction_nav.py --scenario blocked
python run_interaction_nav.py --headless
python run_interaction_nav.py --headless --steps 120
```

`closed` 为默认正常任务；`open` 在任务开始前设置门已打开；`jammed` 用执行器故障注入
保持门关闭；`blocked` 在通道中添加红色实体障碍物。后两项的预期结果是安全停止并报告失败，
不是导航成功。`--steps` 提前结束记录为 `interrupted_or_step_limit`。
无窗口模式仍需要 iGibson 渲染依赖、assets 和可用的图形驱动。

WASD/QE 和鼠标拖动控制观察相机，V 恢复视角，Esc 停止任务并退出。
自主任务期间 O/C/R 不改变门或重置场景，以免人工操作污染任务结果。
任务结束后窗口关闭，终端和日志保留结果；原 `python interaction_scene.py` 观察器仍可独立运行。

## 配置和执行

`interaction_nav.yaml` 的 `scene_config` 相对于配置文件定位；默认继承
`interaction_scene.yaml` 的起终点、场景显示和门参数。导航配置可覆盖这些场景值。
门前等待点在房间 A，门后过渡点在房间 B；成功要求到达最终目标并持续停稳。
任务时间、开门等待和导航超时全部使用仿真时间，与渲染的实际帧率无关。

动作复用 `action_policy.compute_action` 和 `WaypointTracker`，使用最短方向转向。
`forward_action_limit` 和 `crossing_action_limit` 是归一化命令上限，不是 m/s；
Turtlebot 的 DifferentialDriveController 将命令映射到车体速度和轮速。
`angular_action_sign` 默认 +1。完整仿真验收需确认正角速度使机器人朝正 yaw 转动。

地图从 `build_room_urdf()` 的碰撞几何提取墙体和门，排除地面和高处门楣。
门板按实际关节角度旋转；已配置障碍物同时进入物理场景和地图。
机器人采用半径 `robot_radius` 的圆形保守近似，附加 `safety_margin` 和栅格离散余量。
更换机器人时应核实其水平包络并调整半径；当前模型未实现任意高度和外形的三维通行检查。

规划复用 `astar_grid`，禁止对角穿角；路径简化与精确起终点连接均检查净空。
关门时先规划到等待点，开门后再规划至门后过渡点，最后规划至目标。
开门仅发送一次；只有门角度达到 `passage_angle` 且整条穿门走廊满足净空才允许穿门。
穿门过程中若通道失效，立即停止并结束任务，不尝试强行通过或自动恢复。
当前地图覆盖已建模障碍物，不声称检测任意新出现的未知障碍物。

## 显示与日志

窗口分别显示任务阶段、门的实际/目标角度、运动诊断和通道检查结果。
`STALLED` 表示关节未到目标且约 2 秒没有明显角度变化，是诊断提示；
实际失败由任务的开门超时决定。`Passage READY` 表示当前已知几何下的走廊可通行。
右下角地图显示膨胀后的障碍物、规划路径、绿色实际轨迹、机器人和目标。

默认输出至 `outputs/interaction_nav/`，每次运行有独立 JSONL 和 `.summary.json`：

- 每步：动作前后位姿、归一化动作、任务阶段、实际门角度和碰撞接触。
- 汇总：成功与原因、仿真时间、实际行驶距离、开门请求数、等待时间及状态切换。
- 碰撞数：新出现的接触对事件数，排除机器人自身和正常地面支撑；不是持续接触帧数。

## 验证

```bash
python -m pytest -q -p no:cacheprovider
```

纯 Python 测试覆盖地图连通性、全宽净空、状态机停稳条件、四种场景的运动学闭环、
穿门途中失去通道和碰撞过滤。运动学闭环测试不替代物理仿真验收。
`tests/test_interaction_scene.py` 的真实门测试需要 PyBullet，缺失时明确跳过。

在完整 iGibson 环境中还需验收：休眠后的开门动作、渲染同步、机器人落地、
动作方向与速度、无碰撞穿门，以及四种场景是否产生预期日志。
本地缺少 iGibson/PyBullet 时，不将单元测试通过描述为上述物理验收通过。
