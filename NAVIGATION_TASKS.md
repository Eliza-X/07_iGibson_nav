# 多个独立导航任务

`run_navigation_tasks.py` 在当前地图上执行独立任务。每项创建新的环境，
由现有闭环控制器重置机器人并执行一次导航，结束后关闭环境。
未改动 A*、运动策略、场景地图资源或现有地图参数；不支持连续巡航、开门或跨楼层。

## 使用方法

在已安装 iGibson、TurtleBot、Rs 场景资源并注册 `point_nav_fixed` 的 Python 环境中运行。
先生成整批路线预览（不调用 reset/step、不运动机器人）：

```powershell
python run_navigation_tasks.py --plan-only
```

查看终端输出目录内每项的路径 PNG 和 `summary.csv`，再执行：

```powershell
# 全部任务，默认 classic 显示；每项结束自动关闭地图窗口
python run_navigation_tasks.py
# 只运行指定任务
python run_navigation_tasks.py --task existing_forward
# 无窗口批量运行
python run_navigation_tasks.py --headless
# 无窗口录制每项的四宫格视频
python run_navigation_tasks.py --headless --display-mode video
# 每项最多 10 步，仅用于启动检查，不作为导航验收
python run_navigation_tasks.py --short-exec
# 指定清单和输出根目录
python run_navigation_tasks.py --tasks navigation_tasks.yaml --output-dir outputs/batches
```

正式执行也会先完成整批路线检查，再执行通过的任务。批量预览保存为 PNG，
不弹出阻塞式确认窗口；需人工审阅时使用 `--plan-only` 分两次运行。
原有 `python env_nonint_fixed_point.py` 单任务使用方式保持可用。

## 任务配置

`navigation_tasks.yaml` 的 `base_config` 相对于任务清单所在目录解析，
当前引用 `fixed_point_path.yaml`。任务清单只能包含 `base_config` 和 `tasks`。
每项仅允许以下字段，禁止覆盖地图、规划器和控制器参数：

```yaml
- id: room_a_to_room_b
  description: 已人工确认的房间 A 到房间 B
  initial_pos: [0.5, -2.5, 0.02072859]
  initial_orn: [0.0, 0.0, 1.57079632679]
  target_pos: [-0.3, 0.3, -0.00876713]
  execution_max_step: 5000
```

上面的坐标只是现有候选，房间名称不是坐标的已验证语义。
位置必须是三维世界坐标（米），朝向是 RPY（弧度）。编号以字母或数字开头，
只允许 ASCII 字母、数字、下划线和连字符，最多 64 字符，忽略大小写不得重复。
每项必须明确指定起点、初始朝向和终点。

执行预算依次取任务的 `execution_max_step`、公共配置中的该字段、默认 5000。
批处理将预算同步为环境和控制器使用的 `max_step`；`--short-exec` 优先覆盖为 10。
公共配置中的旧 `max_step` 不控制本执行入口。

随附两个任务仅使用已有起终点，分别正向和反向，尚未验证房间归属或真实通行性。
添加房间内、其他房间内部和跨房间任务时，请先在现有地图确认坐标再添加；
不自动采样坐标、不吸附无效终点、不修改地图来强行连通路径。

## 输出与失败处理

每批输出到独立的时间戳加随机编号目录，每项独立保存：

- `config.yaml`：本项配置和执行预算快照（显示开关等派生值由原入口处理）。
- `Rs_floor0_path.png/json`：通过规划检查后的路径图与规划坐标。
- `logs/*.jsonl`：逐步实际位姿、动作、到达事件和结果，可用于还原轨迹。
- `result.json`：执行返回的完整结果；视频模式另有 `videos/`。

批次根目录有 `summary.json` 和可用 Excel 打开的 UTF-8 BOM `summary.csv`，
包含规划是否成功、执行是否成功、原因、规划距离、实际路径长度、步数和产物路径。
实际路径长度取运行任务的 `info.path_length`，任务未提供时留空。
`success_rate` 的分母为已执行任务数，跳过或仅规划的任务不算执行失败。

配置错误在创建环境前拒绝。不可达或端点无法连接的任务标记 `planning_failed` 并跳过；
普通执行异常记录 `execution_error` 后继续下一项。正常结束的任务标记 `completed`，
是否成功以 `execution_success` 为准。环境构造失败或缺少运行依赖终止整批，
未完成任务标记 `batch_aborted`，根目录保存 `fatal_error`。Ctrl+C 同样保存中断状态后退出。
每次规划及执行后刷新汇总，已完成结果不会因后续任务失败被丢弃。

## 验证范围

```powershell
python -m pytest -q -p no:cacheprovider
```

自动测试验证任务选择、地图参数保护、先全量规划后执行、失败隔离、公共失败终止、
输出隔离、配置不被原入口修改，以及地图窗口非阻塞清理。
测试使用模拟环境，不代表已完成 iGibson 物理仿真验收。
当前开发 Python 未安装 iGibson；须在实际运行环境检查门口净空、落地位置、
往返成功与轨迹，并按已确认坐标逐步扩充至计划中的 6 个任务。
