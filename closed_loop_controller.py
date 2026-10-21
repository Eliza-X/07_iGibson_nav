"""Closed-loop execution for fixed-point navigation."""

import numpy as np
from igibson.render.profiler import Profiler


def run_closed_loop(env, config_data, action_fn):
    """Reset and run one episode; the caller remains responsible for closing env.

    action_fn accepts (state, dist_tol, turn_direction) and returns a 2D action.
    config_data supplies dist_tol, max_step, and turn_direction.
    """
    # 重置环境，并保存初始观测
    state = env.reset()

    print("当前任务类型：", type(env.task).__name__)
    print("机器人起始位置：", env.robots[0].get_position())
    print("目标位置：", env.task.target_pos)
    print("动作空间：", env.action_space)

    # 当前控制逻辑要求动作包含两个数：前进指令和转向指令
    if env.action_space.shape != (2,):
        raise ValueError("这段代码要求 TurtleBot 使用二维连续动作。")

    # 从 YAML 中读取成功距离容差和最大步数
    dist_tol = float(config_data.get("dist_tol", 0.36))
    max_steps = int(config_data.get("max_step", 500))

    turn_direction = str(config_data.get("turn_direction", "left")).strip().lower()
    if turn_direction not in ("left", "right"):
        raise ValueError("配置 turn_direction 必须为 left 或 right")
    print("单向旋转模式：", turn_direction)

    for i in range(max_steps):
        # 根据当前观测计算动作，替代随机采样
        action = action_fn(state, dist_tol, turn_direction)

        # 将动作限制在环境允许的范围内
        action = np.clip(
            action,
            env.action_space.low,
            env.action_space.high,
        ).astype(np.float32)

        # 执行动作，并获得更新后的观测
        with Profiler("Environment action step"):
            state, reward, done, info = env.step(action)

        # 每 10 步打印一次执行后的状态
        if (i + 1) % 10 == 0 or done:
            distance = float(state["task_obs"][0])
            angle_deg = np.degrees(float(state["task_obs"][1]))

            print(
                f"step={i + 1}, "
                f"distance={distance:.3f} m, "
                f"angle={angle_deg:.1f} deg, "
                f"turn_direction={turn_direction}, "
                f"action={action}"
            )

        # 由环境判断回合是否结束
        if done:
            print("回合结束，是否成功：", info.get("success"))
            print("最终机器人位置：", env.robots[0].get_position())
            print("任务信息：", info)
            break
    else:
        print("脚本达到步数上限，环境尚未报告结束。")
