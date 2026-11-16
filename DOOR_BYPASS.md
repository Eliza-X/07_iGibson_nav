# 连续电机开门实验

自主流程固定为：接近门前 → 停稳 → 请求开门 → 验证角度和净空 → 低速通过 → 终点停稳。
代价选择暂不启用；开门失败停止，不绕行。旧代价函数保留但不由运行入口调用。

## 使用

```powershell
python run_door_bypass.py --inspect-door
python run_door_bypass.py
python run_door_bypass.py --headless
python run_door_bypass.py --initially-open
python run_door_bypass.py --fail-open
python run_door_bypass.py --preview
```

inspect-door：机器人静止，物理仿真继续，按 O 开门、C 关门；显示实际角度。
左键拖动视角，滚轮缩放，R 恢复视角，Esc 退出。preview 仍为冻结预览。
plan-only 只打印固定流程；open-seconds 参数已移除。

## 实现与验证

参考 feature/interaction 的橙色门板、黄色识别条、惯性及电机唤醒逻辑。
门宽 1.34 m、高 2.2 m；为适配当前隔墙方向，门轴为 (0,-0.67,0.03)，打开角度为 -90 度。
墙高 2.5 m，门洞带过梁。默认速度 0.5 rad/s，驱动力参数 40。
运行期间不使用 resetJointState；直接设置关节仅用于初始化。
地图和物理模型共用几何定义，按实际门角度更新。
通行时检查净空，限制前进动作，并检查全部机器人 links 与门的接触。
日志保存到 outputs/door_bypass/时间戳，含门角度、门扇位姿、碰撞包围盒及接触数。
窗口继续支持等比例缩放和固定文字，默认渲染 1280×960。

本机所检查的 Python 环境不能导入 iGibson，尚未完成物理验收。
自动测试覆盖电机命令、地图阻挡和状态转换，不能代替实际碰撞/渲染验证。
请先手动检查门连续转动且显示同步，再验证关门阻挡、开门通过和失败停止。
