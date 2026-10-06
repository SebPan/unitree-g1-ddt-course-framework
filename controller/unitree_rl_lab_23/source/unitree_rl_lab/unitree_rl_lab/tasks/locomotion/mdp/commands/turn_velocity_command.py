from __future__ import annotations

from dataclasses import MISSING

import torch

from isaaclab.envs.mdp import UniformVelocityCommandCfg
from isaaclab.envs.mdp.commands.velocity_command import UniformVelocityCommand
from isaaclab.utils import configclass


class TurnAwareUniformVelocityCommand(UniformVelocityCommand):
    """Velocity command generator with explicit in-place turning samples.

    Compared with IsaacLab's default UniformVelocityCommand, this command generator
    additionally samples a fixed proportion of pure yaw commands:

        lin_vel_x = 0
        lin_vel_y = 0
        ang_vel_z != 0

    This is used to train in-place left/right turning behavior.
    """

    def _resample_command(self, env_ids):
        # First use the original uniform sampling logic.
        super()._resample_command(env_ids)

        if len(env_ids) == 0:
            return

        cfg = self.cfg
        device = self.device

        # Explicitly divide sampled environments into:
        # 1. standing command
        # 2. in-place turning command
        # 3. normal mixed velocity command
        rand = torch.rand(len(env_ids), device=device)

        standing_mask = rand < cfg.rel_standing_envs
        turn_mask = (rand >= cfg.rel_standing_envs) & (
            rand < cfg.rel_standing_envs + cfg.rel_turn_envs
        )

        standing_env_ids = env_ids[standing_mask]
        turn_env_ids = env_ids[turn_mask]

        # Clear original standing flags produced by super()._resample_command().
        # Otherwise, some in-place turning commands may be overwritten to zero
        # in _update_command().
        self.is_standing_env[env_ids] = False

        if self.cfg.heading_command:
            self.is_heading_env[env_ids] = False

        # Standing command: zero linear and angular velocity.
        if len(standing_env_ids) > 0:
            self.vel_command_b[standing_env_ids, :] = 0.0
            self.is_standing_env[standing_env_ids] = True

        # In-place turning command:
        # zero x/y velocity, nonzero yaw velocity.
        if len(turn_env_ids) > 0:
            self.vel_command_b[turn_env_ids, 0] = 0.0
            self.vel_command_b[turn_env_ids, 1] = 0.0

            low, high = cfg.turn_ang_vel_z
            yaw_cmd = torch.empty(len(turn_env_ids), device=device).uniform_(low, high)

            # Avoid tiny yaw commands that are too close to standing.
            min_abs = cfg.turn_min_abs_ang_vel_z
            yaw_sign = torch.sign(yaw_cmd)
            yaw_sign = torch.where(yaw_sign == 0.0, torch.ones_like(yaw_sign), yaw_sign)

            yaw_cmd = torch.where(
                torch.abs(yaw_cmd) < min_abs,
                yaw_sign * min_abs,
                yaw_cmd,
            )

            self.vel_command_b[turn_env_ids, 2] = yaw_cmd
            self.is_standing_env[turn_env_ids] = False

            if self.cfg.heading_command:
                self.is_heading_env[turn_env_ids] = False


@configclass
class TurnAwareUniformLevelVelocityCommandCfg(UniformVelocityCommandCfg):
    """Configuration for TurnAwareUniformVelocityCommand."""

    class_type: type = TurnAwareUniformVelocityCommand

    # Keep this field because your original Unitree RL Lab wrapper has it.
    limit_ranges: UniformVelocityCommandCfg.Ranges = MISSING

    # Probability of explicit in-place turning commands.
    rel_turn_envs: float = 0.25

    # Yaw command range for in-place turning.
    turn_ang_vel_z: tuple[float, float] = (-0.8, 0.8)

    # Minimum absolute yaw command for in-place turning.
    turn_min_abs_ang_vel_z: float = 0.25