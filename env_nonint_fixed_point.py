"""
执行固定点的导航任务
yaml文件读取这个文件夹中的yaml文件
"""

import logging
import os
from sys import platform

import yaml
import numpy as np

import igibson
from igibson.envs.igibson_env import iGibsonEnv
from igibson.render.profiler import Profiler
from igibson.utils.assets_utils import download_assets, download_demo_data

# 动作计算函数
def compute_action(state, dist_tol, turn_direction):
    """先原地对准目标，再直行；旋转只允许指定方向。"""
    if turn_direction not in ("left", "right"):
        raise ValueError("turn_direction 必须为 left 或 right")

    distance = float(state["task_obs"][0])
    angle = float(state["task_obs"][1])

    # 将目标相对方向角转换到 [-pi, pi]
    angle = np.arctan2(np.sin(angle), np.cos(angle))

    # 已进入目标距离容差，停止运动
    if distance < dist_tol:
        return np.zeros(2, dtype=np.float32)

    angle_tol = 0.3  # 弧度，约 17.2 度

    # 已对准：只前进，不转向
    if abs(angle) <= angle_tol:
        forward_cmd = min(0.2, 0.5 * distance)
        return np.array([forward_cmd, 0.0], dtype=np.float32)

    # 未对准：沿指定方向原地旋转，必要时绕较长的一圈
    # 按正转向指令为左转、负转向指令为右转的约定
    if turn_direction == "left":
        remaining_angle = angle % (2 * np.pi)
        turn_sign = 1.0
    else:
        remaining_angle = (-angle) % (2 * np.pi)
        turn_sign = -1.0

    turn_cmd = turn_sign * min(0.3, 0.8 * remaining_angle)
    return np.array([0.0, turn_cmd], dtype=np.float32)

# 程序主函数
def main(selection="user", headless=False, short_exec=False):
    """
    Creates an iGibson environment from a config file with a turtlebot in Rs (not interactive).
    It steps the environment 100 times with random actions sampled from the action space,
    using the Gym interface, resetting it 10 times.
    """
    print("*" * 80 + "\nDescription:" + main.__doc__ + "*" * 80)
    # If they have not been downloaded before, download assets and Rs Gibson (non-interactive) models
    download_assets()
    download_demo_data()
    # config_filename = os.path.join(igibson.configs_path, "turtlebot_static_nav.yaml")
    # 读取此脚本所在目录中的yaml文件
    config_filename = os.path.join(os.path.dirname(os.path.abspath(__file__)),"turtlebot_static_nav.yaml",)
    print("实际读取的配置文件：", config_filename)
    config_data = yaml.load(open(config_filename, "r"), Loader=yaml.FullLoader)
    # Reduce texture scale for Mac.
    if platform == "darwin":
        config_data["texture_scale"] = 0.5

    # Shadows and PBR do not make much sense for a Gibson static mesh
    config_data["enable_shadow"] = False
    config_data["enable_pbr"] = False

    env = iGibsonEnv(config_file=config_data, mode="gui_interactive" if not headless else "headless")
    max_iterations = 10 if not short_exec else 1

    # 闭环控制器
    try:
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
            action = compute_action(state, dist_tol, turn_direction)

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

    finally:
        env.close()    
    
    # 随机动作代码
    # for j in range(max_iterations):
    #     print("Resetting environment")
    #     env.reset()
    #     for i in range(100):
    #         with Profiler("Environment action step"):
    #             action = env.action_space.sample()
    #             state, reward, done, info = env.step(action)
    #             if done:
    #                 print("Episode finished after {} timesteps".format(i + 1))
    #                 break
    # env.close()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
