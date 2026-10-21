"""
执行固定点的导航任务
yaml文件读取这个文件夹中的yaml文件
"""

import logging
import os
from sys import platform

import yaml

import igibson
from igibson.envs.igibson_env import iGibsonEnv
from igibson.utils.assets_utils import download_assets, download_demo_data

from action_policy import compute_action
from closed_loop_controller import run_closed_loop


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
        run_closed_loop(env=env, config_data=config_data, action_fn=compute_action)
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
