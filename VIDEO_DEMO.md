# 四宫格导航展示

在已安装 iGibson、场景资源、Matplotlib 和 OpenCV 的环境中运行：

```powershell
python env_nonint_fixed_point.py
```

`fixed_point_path.yaml` 默认设置 `display_mode: video`，开启四宫格预览和录像。
一个窗口按左上、右上、左下、右下顺序显示：实时轨迹地图、固定房间机位、
机器人斜后方跟随机位、机器人第一视角。所有图像在同一个 reset/step 后采集，
采集过程不推进仿真；各摄像机渲染后恢复原渲染器的相机状态。

视频保存到脚本目录下 `outputs/videos/navigation_时间_唯一编号.mp4`，使用 MP4V 编码。
每格默认 640×480，总画面 1280×960。地图保持比例留边，标题和状态条显示视角、
环境步数和仿真时间。原始相机渲染分辨率在四宫格模式下也设置为每格尺寸。

帧率取实际 `env.action_timestep` 的倒数：0.1 秒控制步对应 10 FPS。
包含一帧 reset 初始状态、每个环境步一帧以及默认两秒的最终状态停留。
渲染变慢会降低预览的墙钟速度，但视频仍按仿真时间播放。
不为提高帧率改变控制步长，不生成插值运动帧。

## 配置

```yaml
display_mode: video  # video 或 classic
video_output_dir: outputs/videos
video_panel_width: 640
video_panel_height: 480
video_final_hold_seconds: 2.0
fixed_camera_position: [1.3, -3.0, 2.2]
fixed_camera_target: [0.2, -1.2, 0.25]
```

固定相机位置和注视点为世界 XYZ，单位米。当前坐标是针对当前路线的初始候选，
未经过实际 Rs 场景视线验证。预览右上格后调整；墙壁遮挡不会被自动消除，
跨房间的路线不能保证始终被单个固定机位看见。

后方机位沿用后退 1.5 米、左偏 0.6 米、抬高 1.2 米，注视机器人上方 0.25 米。
四宫格模式始终包含跟随格和地图格；原入口的 `follow_camera`、`show_route_map`
只控制旧的独立窗口模式，`show_map_waypoints` 仍控制地图格的紫色路点。

- `display_mode: classic`：恢复原来的两个 Viewer 与独立地图窗口，不录像。
- `display_mode: video`：四宫格预览并保存 MP4。录像参数只在该模式使用。
- 按 Esc 或关闭四宫格窗口：隐藏预览，控制循环和录像继续。
- 导航结束：添加结果画面，完成录像并关闭资源。
- 异常或 Ctrl+C：释放视频写入器，保留已写入的帧，不伪造成功结果。

不弹窗录制（仍需可用的 iGibson 离屏渲染驱动）：

```powershell
python env_nonint_fixed_point.py --display-mode video --headless
```

OpenCV 缺失时在实际运行环境安装 `opencv-python`；仅有 headless 版本的 OpenCV
可以编码视频，但不能提供四宫格预览窗口。若 MP4 编码器不可用，程序会明确报错。

自动测试使用模拟渲染器检查相机恢复、颜色、布局、视频时序和 MP4 解码；
实际场景加载、第一视角及固定机位遮挡仍需在完整 iGibson 环境验收。

## 模式选择和参数优先级

```powershell
# 使用 YAML 中的 display_mode（当前默认 video）
python env_nonint_fixed_point.py
# 临时使用四宫格录像模式，不修改 YAML
python env_nonint_fixed_point.py --display-mode video
# 临时使用原有展示模式
python env_nonint_fixed_point.py --display-mode classic
# 原有展示模式下关闭跟随相机、独立地图或地图路点
python env_nonint_fixed_point.py --display-mode classic --no-follow-camera --no-route-map --no-map-waypoints
```

命令行 `--display-mode` 优先于 YAML；缺少 YAML 字段时默认 classic。
Python 调用也可以使用 `main(display_mode="classic")` 覆盖配置。
旧的 `quad_view_enabled`、`record_video` 不再作为入口模式开关，请使用 `display_mode`。
启动时打印最终模式、是否录像和是否预览。classic 配合 `--headless` 时既不弹窗也不录像；
video 配合 `--headless` 时不弹窗但仍录像。`--short-exec` 可将任务限制为 10 步。
