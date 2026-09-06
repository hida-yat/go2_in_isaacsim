# Go2 flat-terrain policy controller, adapted from
# isaacsim.robot.policy.examples's SpotFlatTerrainPolicy, but wired up to a
# policy trained with unitree_rl_lab (Isaac Lab) instead of the shipped Spot policy.
#
# Observation / action layout below mirrors unitree_rl_lab's exported
# params/deploy.yaml for this checkpoint (same file the real-robot C++ deploy
# stack in unitree_rl_lab/deploy consumes):
#
#   observations.policy (45-dim, in this order):
#     base_ang_vel        (3,  scale 0.2)   -- body-frame angular velocity
#     projected_gravity   (3,  scale 1.0)   -- gravity vector in body frame
#     velocity_commands   (3,  scale 1.0)   -- (lin_vel_x, lin_vel_y, ang_vel_z)
#     joint_pos_rel       (12, scale 1.0)   -- joint_pos - default_joint_pos
#     joint_vel_rel       (12, scale 0.05)  -- joint_vel - default_joint_vel
#     last_action         (12, scale 1.0)   -- previous raw policy output
#
#   action: JointPositionAction, scale 0.25, offset = default_joint_pos

from typing import Optional

import numpy as np
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.policy.examples.controllers import PolicyController

from . import settings

_NUM_JOINTS = 12
_ANG_VEL_SCALE = 0.2
_JOINT_VEL_SCALE = 0.05
_ACTION_SCALE = 0.25


class Go2FlatTerrainPolicy(PolicyController):
    """The Unitree Go2 quadruped, running a unitree_rl_lab-trained flat terrain policy."""

    def __init__(
        self,
        prim_path: str,
        root_path: Optional[str] = None,
        name: str = "go2",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        """
        Initialize robot and load the RL policy.

        Args:
            prim_path (str) -- prim path of the robot on the stage
            root_path (Optional[str]): The path to the articulation root of the robot
            name (str) -- name of the quadruped
            usd_path (str) -- robot usd filepath. Defaults to the "robot_usd_path" setting
                (Window > Go2 Policy Example > Settings), which itself defaults to
                unitree_model/Go2/usd/go2.usd.
            position (np.ndarray) -- position of the robot
            orientation (np.ndarray) -- orientation of the robot
        """
        if usd_path is None:
            usd_path = settings.get("robot_usd_path")

        super().__init__(name, prim_path, root_path, usd_path, position, orientation)

        self.load_policy(settings.get("policy_path"), settings.get("policy_env_path"))
        self._action_scale = _ACTION_SCALE
        self._previous_action = np.zeros(_NUM_JOINTS)
        self._policy_counter = 0

    def _compute_observation(self, command: np.ndarray) -> np.ndarray:
        """
        Compute the 45-dim observation vector for the policy.

        Argument:
        command (np.ndarray) -- the robot command (v_x, v_y, w_z)

        Returns:
        np.ndarray -- The observation vector.
        """
        ang_vel_I = self.robot.get_angular_velocity()
        _, q_IB = self.robot.get_world_pose()

        R_IB = quat_to_rot_matrix(q_IB)
        R_BI = R_IB.transpose()
        ang_vel_b = np.matmul(R_BI, ang_vel_I)
        gravity_b = np.matmul(R_BI, np.array([0.0, 0.0, -1.0]))

        current_joint_pos = self.robot.get_joint_positions()
        current_joint_vel = self.robot.get_joint_velocities()

        obs = np.zeros(45)
        obs[0:3] = ang_vel_b * _ANG_VEL_SCALE
        obs[3:6] = gravity_b
        obs[6:9] = command
        obs[9:21] = current_joint_pos - self.default_pos
        obs[21:33] = (current_joint_vel - self.default_vel) * _JOINT_VEL_SCALE
        obs[33:45] = self._previous_action

        return obs

    def forward(self, dt: float, command: np.ndarray) -> None:
        """
        Compute the desired joint positions and apply them to the articulation.

        Argument:
        dt (float) -- Timestep update in the world.
        command (np.ndarray) -- the robot command (v_x, v_y, w_z)
        """
        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation(command)
            self.action = self._compute_action(obs)
            self._previous_action = self.action.copy()

        action = ArticulationAction(joint_positions=self.default_pos + (self.action * self._action_scale))
        self.robot.apply_action(action)

        self._policy_counter += 1
