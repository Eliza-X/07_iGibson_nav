# 固定起终点路径图

在已经安装 iGibson、对应依赖与场景资源的 Python 环境中运行：

```powershell
python plot_fixed_point_path.py
```

程序读取脚本旁的 `fixed_point_path.yaml`，加载 Rs 场景，调用
`env.scene.get_shortest_path(..., entire_path=True)`，保存图片和路径数据，
释放仿真环境后自动弹出图片窗口。Matplotlib 窗口保持到手动关闭；
Windows 的非交互 Matplotlib 后端会改用系统图片查看器打开 PNG。
Linux 桌面需要可用的 Matplotlib Tk/Qt 后端及显示服务。

```powershell
# 服务器只保存，不弹窗
python plot_fixed_point_path.py --no-show
# 指定另一份完整配置
python plot_fixed_point_path.py --config my_path.yaml
```

输出默认放在配置文件所在目录的 `outputs/path_planning/`：

- `Rs_floor0_path.png`：白色为导航图保留的可通行区域，黑色为其他区域；
  绿点为起点所在栅格，红星为终点所在栅格，蓝线为场景接口返回路径。
- `Rs_floor0_path.json`：原始起终点、世界坐标 XY 路径、地图行列坐标、
  地图分辨率及接口返回的测地距离（米）。同场景同楼层重跑覆盖这两个文件。

修改 `scene_id`、`floor_num`、`initial_pos`、`target_pos` 可选择规划任务。
当前仅支持 `scene: gibson` 静态室内场景；必须启用 `build_graph: true`。
配置中的默认坐标来自仓库原有注释，尚未在当前机器的 Rs 数据上验证。
若坐标不在该楼层导航图中，程序明确报错，不吸附到最近可通行点。
XY 用于规划，XYZ 中的 Z 只作为原始配置保存；楼层由 `floor_num` 指定。

程序将 `waypoint_resolution` 设为 `trav_map_resolution`，并检查实际场景
`waypoint_interval == 1`，保留所有路径节点，防止稀疏折线掩盖拐角。
图的坐标轴使用地图像素；世界坐标通过场景接口逐点转换为行列坐标。
起终点会量化到栅格，不添加原始坐标到栅格节点的额外连接线。
图呈现的是可通行图最短路径，不代表机器人运动或物理碰撞验证结果。

环境初始化会加载机器人，但入口不调用 `env.reset()` 或 `env.step()`，
不调用运动控制器，也不依赖本仓库任务文件被安装包加载。
Rs 资源使用现有项目的下载辅助函数准备，其他场景需预先安装。
headless 环境加载仍可能需要 iGibson 的图形驱动/EGL 支持。

验证命令：`python -m pytest -q -p no:cacheprovider`。
测试使用模拟场景验证接口参数、行列转换、绘图、导出及弹窗调用；
实际 Rs 地图和桌面窗口仍需在已安装 iGibson 的运行环境中验收。
