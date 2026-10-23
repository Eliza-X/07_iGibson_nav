"""
执行固定点的导航任务
yaml文件读取这个文件夹中的yaml文件
"""

import logging
import os
from sys import platform

import yaml

from igibson.envs.igibson_env import iGibsonEnv
from igibson.utils.assets_utils import download_assets, download_demo_data

from action_policy import compute_action
from closed_loop_controller import run_closed_loop


# 程序主函数
def main(selection="user", headless=False, short_exec=False):
    """
    Run one TurtleBot episode following configured waypoints or a single final goal.
    headless disables the GUI; short_exec limits the controller to 10 steps.
    """
    print("*" * 80 + "\nDescription:" + main.__doc__ + "*" * 80)
    # 读取此脚本所在目录中的yaml文件
    config_filename = os.path.join(os.path.dirname(os.path.abspath(__file__)),"turtlebot_static_nav.yaml",)
    print("实际读取的配置文件：", config_filename)
    with open(config_filename, "r", encoding="utf-8") as config_file:
        config_data = yaml.safe_load(config_file)
    download_assets()
    if config_data.get("scene") == "gibson" and config_data.get("scene_id") == "Rs":
        download_demo_data()
    # Reduce texture scale for Mac.
    if platform == "darwin":
        config_data["texture_scale"] = 0.5

    # Shadows and PBR do not make much sense for a Gibson static mesh
    config_data["enable_shadow"] = False
    config_data["enable_pbr"] = False

    env = iGibsonEnv(config_file=config_data, mode="gui_interactive" if not headless else "headless")

    # 闭环控制器
    try:
        controller_config = dict(config_data)
        if short_exec:
            controller_config["max_step"] = min(10, int(config_data.get("max_step", 500)))
        return run_closed_loop(env=env, config_data=controller_config, action_fn=compute_action)
    finally:
        env.close()

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
