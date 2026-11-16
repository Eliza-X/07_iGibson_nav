# 单门捷径与绕行实验

本实验从 MiniGrid 的 `interaction_cost_planner.py`、`cost_model.py` 和
`replanning_loop.py` 迁移代价比较及反馈机制，没有导入源项目模块。
现有 Rs 入口、任务类和配置未修改。

## 运行

在项目根目录、已安装 iGibson 及 TurtleBot 资产的 Python 环境中运行：

```powershell
python run_door_bypass.py --plan-only
python run_door_bypass.py --headless
python run_door_bypass.py --headless --open-seconds 100
python run_door_bypass.py --headless --initially-open
python run_door_bypass.py --headless --fail-open
```

去掉 `--headless` 使用 iGibson GUI。`--plan-only` 只验证几何和代价，不代表物理验证。
原生 `Viewer` 和 `RobotView` 使用 OpenCV 的 `WINDOW_NORMAL | WINDOW_KEEPRATIO`
模式，可拖动边框调整尺寸，画面保持比例。Rs 的 classic 入口也启用相同适配。
此适配仅在创建环境期间改变这两个窗口的创建参数，保留 iGibson 原有鼠标操作。
每次实际运行在 `outputs/door_bypass/时间戳` 保存配置、URDF 和 JSONL 日志。
不自动下载资产或修改安装的 iGibson。

## 只观察场景

```powershell
python run_door_bypass.py --preview
python run_door_bypass.py --preview --initially-open
```

预览加载并重置一次环境后冻结物理状态，不运行导航、开门策略或 episode 循环。
使用独立 Tk 窗口（需要 tkinter 和 Pillow），不打开原生查看器。
鼠标拖动窗口边框或最大化时，图像等比例放大/缩小，多余区域留黑边，不拉伸。
左键拖动画面旋转观察视角，滚轮缩放，R 恢复视角，Esc 或关闭按钮退出。
画面以 1280×960 渲染后缩放显示；扩大窗口不额外增加三维渲染像素。
`--preview` 不能与 `--headless` 或 `--plan-only` 同用。

## 接口与实验约定

- `planning.py`：米制 XY / 行列转换、带净空的地图、完整路线时间估计。
- `scene.py`：同一墙体定义生成物理场景；独立关节门用 iGibson 对象导入渲染器。
- `runtime.py`：通过环境子类加载新任务；唯一 runner 调用 reset 和 step。
- 复用当前 `astar_grid`、`WaypointTracker`、`compute_action`，不复用整段 episode 控制器。
- 假设开门地图只用于估价；真实地图由实际门关节角度生成。
- 软件开门是等待指定服务时间后直接设置关节，不模拟机械臂或门的连续扫掠过程。
  触发点位于门扇扫掠范围外。开门后的占据仍包含门扇。
- 第一版单层、固定几何、单门、无箱子；碰撞终止，卡住可重规划，开门失败转绕行。
- 到达终点才成功，门前目标不修改任务终点；不计算动态场景定义尚未确定的 SPL。
- `estimated_speed` 是代价模型标定参数，不是归一化底盘动作的物理速度保证。
  `clearance_radius` 应覆盖实际机器人外形及安全余量。

## 验证边界

开发机器可读取 `D:/Projects/A_iGibson/iGibson` 源码，但检查时当前 Python
及 `D:/AnacondaInstall/envs/igibson/python.exe` 均不能导入 igibson。
因此目前完成规划测试与运动学替身闭环测试，尚未完成真实 iGibson 仿真验收。
首次实际运行需核实动作符号、机器人尺寸、门的渲染/碰撞及导航耗时。
运动学替身测试不验证物理接触、碰撞或渲染。
