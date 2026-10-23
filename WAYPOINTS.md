# 手动 waypoint 跟踪

默认配置在空场景执行 `(0,0) → (1,0) → (1,1)`。在已配置好 iGibson、TurtleBot 资源及 `point_nav_fixed` 任务的环境中运行：

```powershell
python env_nonint_fixed_point.py
```

无 GUI 执行：

```powershell
python -c "from env_nonint_fixed_point import main; main(headless=True)"
```

`main(short_exec=True)` 只运行最多 10 步，用于启动检查，不用于路线验收。

## 配置

- `waypoints`：非空的世界坐标 XY 列表，最后一点必须等于 `target_pos` 的 XY。删除整个字段可恢复单点导航；空列表视为配置错误。
- `waypoint_tolerance`：中间点到达容差，默认 0.15 米。
- 最后一点使用实际 `env.task.dist_tol`，距离严格小于容差才算到达。
- `turn_direction`：保留 `left` / `right` 固定转向策略及原动作大小。归一化动作不直接等于米/秒。
- `visible_waypoints`：交互 GUI 下显示绿色待到达点和金色当前点，已完成点隐藏。与场景最短路径显示无关。
- `waypoint_log_dir`：默认 `logs`；相对路径以脚本目录为基准，每次运行生成独立 JSONL 文件。

控制器不修改最终任务目标、任务观测、奖励或终止条件。路线应避免在完成中间点前穿过最终目标成功范围。`accepted=true` 要求环境成功且序列完成；提前终点成功会记录为 `early_final_goal`。

## 日志与验收

日志包含起始配置、逐步记录和最终汇总。索引从 0 开始。`pose_before`、`distance`、`angle_error`、`waypoint_index` 和 `action` 属于动作执行前；`pose_after`、`reached_after` 属于执行后。角度单位为弧度。`reached_before` 记录起点就在容差范围内等情况。

结果区分 `success`、`early_final_goal`、`environment_done` 和 `controller_step_limit`。环境提供的详细信息原样保留，不猜测失败原因。序列提前完成而环境尚未结束时发送零动作，继续等待环境判定或步数上限。

测试命令：

```powershell
python -m pytest -q -p no:cacheprovider
```

测试覆盖单点、直线多点、L 形和左右转折路线，分别使用固定左转和右转。闭环路线测试使用理想平面运动模型，只用于逻辑验证，不代表 iGibson 物理仿真已通过。

真实仿真还需检查：GUI 目标显示、角速度正负方向、最终成功判定的距离边界、归一化动作映射，以及日志中按顺序到达每个点并 `accepted=true`。

## 任务加载

入口仍由 `iGibsonEnv` 根据 `task: point_nav_fixed` 加载任务，并在闭环启动时打印实际任务类文件路径。本仓库的 `point_nav_fixed_task.py` 没有被入口直接导入；运行环境必须已有相应任务注册。本次跟踪及标记在控制器侧实现，无须修改已安装的任务类。

当前开发 Python 环境没有安装 iGibson，因此尚未核实真实任务注册路径、仿真效果或 GUI 渲染。若加载任务失败，应先依据运行环境的 iGibson 版本确认注册方式。
