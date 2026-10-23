"""Action policy for fixed-point navigation."""

import numpy as np


# 动作计算函数
def compute_action(distance, angle_error, dist_tol, turn_direction):
    """先原地对准目标，再直行；旋转只允许指定方向。"""
    if turn_direction not in ("left", "right"):
        raise ValueError("turn_direction 必须为 left 或 right")

    distance = float(distance)
    angle = float(angle_error)

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
