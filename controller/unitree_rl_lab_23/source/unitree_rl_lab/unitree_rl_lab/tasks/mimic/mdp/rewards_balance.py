from __future__ import annotations

import torch
from isaaclab.managers import SceneEntityCfg


def _quat_to_roll_pitch(quat_w: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    """Convert quaternion (w, x, y, z) to roll and pitch."""
    qw = quat_w[..., 0]
    qx = quat_w[..., 1]
    qy = quat_w[..., 2]
    qz = quat_w[..., 3]

    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = torch.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = torch.clamp(sinp, -1.0, 1.0)
    pitch = torch.asin(sinp)

    return roll, pitch


def base_roll_pitch_l2(env) -> torch.Tensor:
    """Penalize base roll/pitch magnitude.

    Main purpose:
        suppress persistent lateral leaning during support phases.
    """
    robot = env.scene["robot"]
    root_quat_w = robot.data.root_quat_w

    roll, pitch = _quat_to_roll_pitch(root_quat_w)

    # roll更重要，pitch次之
    return roll.square() + 0.5 * pitch.square()


def base_roll_ang_vel_l2(env) -> torch.Tensor:
    """Penalize base roll angular velocity.

    Main purpose:
        reduce sudden side-fall tendency during single-leg support.
    """
    robot = env.scene["robot"]
    base_ang_vel = robot.data.root_ang_vel_b  # (num_envs, 3), body frame
    roll_ang_vel = base_ang_vel[:, 0]
    return roll_ang_vel.square()


def feet_force_balance_l1(env, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize large left-right vertical contact force imbalance.

    Important:
        This version is ONLY active when both feet are clearly in contact.
        That prevents it from interfering with valid single-leg support.
    """
    contact_sensor = env.scene.sensors[sensor_cfg.name]

    if len(sensor_cfg.body_ids) != 2:
        raise ValueError(
            f"feet_force_balance_l1 expects exactly 2 body ids, got {len(sensor_cfg.body_ids)}."
        )

    left_id, right_id = sensor_cfg.body_ids
    net_forces_w = contact_sensor.data.net_forces_w  # (num_envs, num_bodies, 3)

    left_fz = torch.relu(net_forces_w[:, left_id, 2])
    right_fz = torch.relu(net_forces_w[:, right_id, 2])

    # 关键修改：只有双脚都明确承重时才激活
    contact_threshold = 20.0
    active_mask = (left_fz > contact_threshold) & (right_fz > contact_threshold)

    total_fz = left_fz + right_fz
    diff_fz = torch.abs(left_fz - right_fz)
    normalized_diff = diff_fz / (total_fz + 1.0e-6)

    penalty = torch.where(active_mask, normalized_diff, torch.zeros_like(normalized_diff))
    return penalty


def pelvis_lateral_drift_l2(env, command_name: str) -> torch.Tensor:
    """Penalize pelvis lateral position drift relative to reference motion.

    Main purpose:
        reduce side drifting during support-leg switching and single-leg stance.
    """
    robot = env.scene["robot"]
    command = env.command_manager.get_command(command_name)

    # command中前面是参考动作相关量，具体组织由mimic command给出；
    # 这里不直接依赖其内部结构，而是使用环境现成的pelvis刚体位置。
    pelvis_id = robot.find_bodies(["pelvis"], preserve_order=True)[0][0]

    pelvis_pos_w = robot.data.body_pos_w[:, pelvis_id, :]
    root_pos_w = robot.data.root_pos_w

    # 用 pelvis 相对 root 的局部横向位移近似约束
    # 对23DoF问题，重点关心 y 方向的漂移
    lateral_offset = pelvis_pos_w[:, 1] - root_pos_w[:, 1]

    return lateral_offset.square()