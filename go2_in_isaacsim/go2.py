# Policy controller adapted from isaacsim.robot.policy.examples's
# SpotFlatTerrainPolicy, but wired up to a unitree_rl_lab (Isaac Lab) policy
# instead of the shipped Spot policy.
#
# Unlike the Spot example, the observation vector and action scale/offset are
# NOT hardcoded here. They're built at runtime from unitree_rl_lab's exported
# params/deploy.yaml (the same file its C++ real-robot deploy stack reads),
# which records the observation term list/order/scales and the action
# scale/offset/clip for that specific checkpoint. This means retraining with
# a different unitree_rl_lab environment -- different terrain/curriculum, a
# different scale, even a different robot with a different joint count --
# works by just pointing "Policy deploy.yaml" (and the matching .pt/env.yaml)
# at the new checkpoint, no code changes, *as long as* the checkpoint only
# uses observation terms this file knows how to compute (see
# _OBSERVATION_TERMS below -- proprioceptive terms computable from the bare
# articulation state; nothing that needs an extra sensor, like height_scan).
#
# Joint PD gains / default pose / effort & velocity limits still come from
# the Isaac Lab params/env.yaml dump via the base PolicyController class,
# matched by joint *name* pattern against the robot's actual USD joint names
# -- see isaacsim.robot.policy.examples.controllers.config_loader -- so that
# part is already order-independent. The observation/action arrays below are
# positional (as unitree_rl_lab exports them), so they rely on this
# extension's robot USD having the same native joint order as the USD used
# for training (true as long as you don't hand-reorder joints in the USD).

import io
from typing import Optional

import numpy as np
import omni.client
import yaml
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.policy.examples.controllers import PolicyController
from isaacsim.robot.policy.examples.controllers.config_loader import get_robot_joint_properties
from omni.physx import get_physx_simulation_interface

from . import settings


def _load_yaml(path: str) -> dict:
    file_content = omni.client.read_file(path)[2]
    file = io.BytesIO(memoryview(file_content).tobytes())
    return yaml.safe_load(file)


def _leg_dof_indices(dof_names, policy_env_params: dict) -> list:
    """Indices into dof_names for Go2's own 12 leg joints, excluding any
    extra DOFs merged into the *same* PhysX articulation from an attachment
    like Piper (see tools/build_go2_with_mid360_and_piper.py: PhysX doesn't
    allow two independent articulations to stay independent once a joint
    rigidly connects them -- they fuse into one, so an arm welded to Go2's
    base ends up contributing DOFs to this exact self.robot.dof_names list).
    This checkpoint's observation/action arrays are positional and sized to
    exactly 12 (its own training-time joint count), so every place that
    reads/writes joint state by dof index must go through this filter
    instead of touching all of self.robot.dof_names.

    Deliberately NOT matched via the actuators' own joint_names_expr (like
    isaacsim.robot.policy.examples.controllers.config_loader does): Go2's
    checkpoint declares a single actuator group with joint_names_expr:
    ['.*'] (unambiguous at training time, when the robot really only had 12
    joints), which would match Piper's joints too. joint_sdk_names is
    unitree_rl_lab's own export of the real robot's actual actuated joint
    names (in Unitree SDK order, not necessarily this USD's dof order --
    used here only as a membership set, so self.robot.dof_names' own
    relative order -- confirmed to match the bare Go2 USD's order once
    non-member joints are dropped -- is preserved)."""
    leg_names = set(policy_env_params["scene"]["robot"]["joint_sdk_names"])
    return [i for i, joint in enumerate(dof_names) if joint in leg_names]


def _term_base_lin_vel(policy: "Go2FlatTerrainPolicy", command: np.ndarray) -> np.ndarray:
    return policy._to_body_frame(policy.robot.get_linear_velocity())


def _term_base_ang_vel(policy: "Go2FlatTerrainPolicy", command: np.ndarray) -> np.ndarray:
    return policy._to_body_frame(policy.robot.get_angular_velocity())


def _term_projected_gravity(policy: "Go2FlatTerrainPolicy", command: np.ndarray) -> np.ndarray:
    return policy._to_body_frame(np.array([0.0, 0.0, -1.0]))


def _term_velocity_commands(policy: "Go2FlatTerrainPolicy", command: np.ndarray) -> np.ndarray:
    return np.asarray(command, dtype=np.float64)


def _term_joint_pos_rel(policy: "Go2FlatTerrainPolicy", command: np.ndarray) -> np.ndarray:
    return policy.robot.get_joint_positions(joint_indices=policy._leg_dof_indices) - policy.default_pos


def _term_joint_pos(policy: "Go2FlatTerrainPolicy", command: np.ndarray) -> np.ndarray:
    return policy.robot.get_joint_positions(joint_indices=policy._leg_dof_indices)


def _term_joint_vel_rel(policy: "Go2FlatTerrainPolicy", command: np.ndarray) -> np.ndarray:
    return policy.robot.get_joint_velocities(joint_indices=policy._leg_dof_indices) - policy.default_vel


def _term_joint_vel(policy: "Go2FlatTerrainPolicy", command: np.ndarray) -> np.ndarray:
    return policy.robot.get_joint_velocities(joint_indices=policy._leg_dof_indices)


def _term_last_action(policy: "Go2FlatTerrainPolicy", command: np.ndarray) -> np.ndarray:
    return policy._previous_action.copy()


# Proprioceptive unitree_rl_lab / Isaac Lab mdp observation terms computable
# from the bare articulation state (no extra sensors). Extend this if you
# retrain with another term from this family; anything that needs a sensor
# (height_scan, camera, ...) is out of scope for this simple example.
_OBSERVATION_TERMS = {
    "base_lin_vel": _term_base_lin_vel,
    "base_ang_vel": _term_base_ang_vel,
    "projected_gravity": _term_projected_gravity,
    "velocity_commands": _term_velocity_commands,
    "joint_pos_rel": _term_joint_pos_rel,
    "joint_pos": _term_joint_pos,
    "joint_vel_rel": _term_joint_vel_rel,
    "joint_vel": _term_joint_vel,
    "last_action": _term_last_action,
}


class Go2FlatTerrainPolicy(PolicyController):
    """A unitree_rl_lab-trained quadruped policy, driven generically from its exported deploy.yaml."""

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
                (Edit > Preferences > Go2 Policy Example), which itself defaults to the
                bundled unitree_model/Go2/usd/go2.usd.
            position (np.ndarray) -- position of the robot
            orientation (np.ndarray) -- orientation of the robot
        """
        if usd_path is None:
            usd_path = settings.get("robot_usd_path")

        super().__init__(name, prim_path, root_path, usd_path, position, orientation)

        self.load_policy(settings.get("policy_path"), settings.get("policy_env_path"))

        deploy_cfg = _load_yaml(settings.get("policy_deploy_path"))
        # dict preserves insertion order -> this is the exact order the policy's
        # observation vector was concatenated in at training/export time.
        self._obs_spec = list(deploy_cfg["observations"].items())
        action_cfg = next(iter(deploy_cfg["actions"].values()))
        self._action_scale = np.asarray(action_cfg["scale"], dtype=np.float64)
        action_clip = action_cfg.get("clip")
        self._action_clip_lo = np.array([c[0] for c in action_clip]) if action_clip else None
        self._action_clip_hi = np.array([c[1] for c in action_clip]) if action_clip else None

        self._previous_action = np.zeros(len(self._action_scale))
        self._policy_counter = 0

    def initialize(
        self,
        physics_sim_view=None,
        effort_modes: str = "force",
        control_mode: str = "position",
        set_gains: bool = True,
        set_limits: bool = True,
        set_articulation_props: bool = True,
    ) -> None:
        """Same as PolicyController.initialize(), except every joint-property
        write (gains, effort/velocity limits, default_pos/default_vel) is
        scoped to self._leg_dof_indices (Go2's own 12 leg joints) instead of
        the whole articulation -- see _leg_dof_indices()'s docstring for why:
        a Piper arm welded onto the same robot USD (go2_with_mid360_and_piper.usd)
        adds DOFs to this *same* self.robot.dof_names list, and the base
        implementation would otherwise zero out Piper's own pre-authored
        drive gains/effort limits for every joint name that doesn't match
        one of Go2's own actuator patterns, leaving it undrivable."""
        self.robot.initialize(physics_sim_view=physics_sim_view)
        self.robot.get_articulation_controller().set_effort_modes(effort_modes)

        # TODO: Must flush when FSD is enabled.
        # Otherwise the delayed FSD handling next frame will overwrite set_max_efforts below
        get_physx_simulation_interface().flush_changes()

        self.robot.get_articulation_controller().switch_control_mode(control_mode)

        self._leg_dof_indices = _leg_dof_indices(self.robot.dof_names, self.policy_env_params)
        leg_dof_names = [self.robot.dof_names[i] for i in self._leg_dof_indices]
        max_effort, max_vel, stiffness, damping, default_pos, default_vel = get_robot_joint_properties(
            self.policy_env_params, leg_dof_names
        )
        self.default_pos = np.asarray(default_pos)
        self.default_vel = np.asarray(default_vel)

        if set_gains:
            self.robot._articulation_view.set_gains(stiffness, damping, joint_indices=self._leg_dof_indices)
        if set_limits:
            self.robot._articulation_view.set_max_efforts(max_effort, joint_indices=self._leg_dof_indices)

            # TODO: Must flush when FSD is enabled.
            # Otherwise the delayed FSD handling next frame will overwrite set_max_efforts below
            get_physx_simulation_interface().flush_changes()

            self.robot._articulation_view.set_max_joint_velocities(max_vel, joint_indices=self._leg_dof_indices)
        if set_articulation_props:
            self._set_articulation_props()

    def set_default_state(self) -> None:
        """Sets self.robot's post-reset default joint state. Only Go2's own
        leg joints (self._leg_dof_indices) get this checkpoint's default_pos
        -- self.robot.set_joints_default_state() needs a value for *every*
        DOF (no joint_indices filter), so any other DOFs merged into this
        same articulation (e.g. a welded-on Piper arm, see
        _leg_dof_indices()'s docstring) keep whatever position they're
        currently at (their own USD-authored rest pose, untouched so far)
        instead of being silently zeroed."""
        full_default_pos = self.robot.get_joint_positions()
        full_default_pos[self._leg_dof_indices] = self.default_pos
        self.robot.set_joints_default_state(positions=full_default_pos)

    def _to_body_frame(self, vec_world: np.ndarray) -> np.ndarray:
        _, q_IB = self.robot.get_world_pose()
        r_ib = quat_to_rot_matrix(q_IB)
        return np.matmul(r_ib.transpose(), vec_world)

    def _compute_observation(self, command: np.ndarray) -> np.ndarray:
        """
        Compute the observation vector for the policy, built from params/deploy.yaml's
        "observations" section (term list/order/scale/clip) rather than a fixed layout.

        Argument:
        command (np.ndarray) -- the robot command (v_x, v_y, w_z)

        Returns:
        np.ndarray -- The observation vector.
        """
        chunks = []
        for term_name, spec in self._obs_spec:
            term_fn = _OBSERVATION_TERMS.get(term_name)
            if term_fn is None:
                raise NotImplementedError(
                    f"Observation term '{term_name}' from {settings.get('policy_deploy_path')} has no "
                    f"implementation in go2.py. Supported terms: {', '.join(sorted(_OBSERVATION_TERMS))}."
                )
            value = np.atleast_1d(np.asarray(term_fn(self, command), dtype=np.float64))
            value = value * np.asarray(spec.get("scale", 1.0), dtype=np.float64)
            clip = spec.get("clip")
            if clip:
                value = np.clip(value, clip[0], clip[1])
            chunks.append(value)
        return np.concatenate(chunks)

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

        joint_positions = self.default_pos + self.action * self._action_scale
        if self._action_clip_lo is not None:
            joint_positions = np.clip(joint_positions, self._action_clip_lo, self._action_clip_hi)

        self.robot.apply_action(
            ArticulationAction(joint_positions=joint_positions, joint_indices=self._leg_dof_indices)
        )

        self._policy_counter += 1
