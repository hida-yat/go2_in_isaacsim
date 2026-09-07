# Persistent, user-editable configuration for this extension.
#
# Values live under carb's persistent settings tree, so once changed from the
# Settings window (Window > Go2 Policy Example > Settings) they are written to
# the user's own Kit user.config.json and survive restarts / extension
# updates. Nothing here needs to be edited in code to point this extension at
# a different robot USD (e.g. a go2.usd with sensors + an Action Graph added),
# a different environment USD, or a different trained checkpoint.

import os

import carb.settings

_ROOT = "/persistent/exts/go2_in_isaacsim"

# Everything under data/ ships inside this extension (git repo) so it works
# out of the box on a fresh checkout, with no external paths and no
# configuration at all.
_EXT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
# Public alias -- other modules (mid360.py) need the extension root too, to
# point the RTX lidar plugin's profile search path at our bundled configs.
EXT_ROOT = _EXT_ROOT
_DEFAULT_POLICY_PATH = os.path.join(_EXT_ROOT, "data", "Policies", "Go2", "policy.pt")
_DEFAULT_POLICY_ENV_PATH = os.path.join(_EXT_ROOT, "data", "Policies", "Go2", "env.yaml")
_DEFAULT_POLICY_DEPLOY_PATH = os.path.join(_EXT_ROOT, "data", "Policies", "Go2", "deploy.yaml")

# Bundled bare Go2 USD (no sensors). Point "robot_usd_path" at a
# sensor-equipped variant (e.g. a go2_with_sensors.usd containing
# cameras/lidar + an Action Graph publishing them) to swap it in without
# touching any code.
_DEFAULT_ROBOT_USD_PATH = os.path.join(_EXT_ROOT, "data", "Robots", "Go2", "usd", "go2.usd")

# Empty by default -> the example just spawns a flat ground plane.
# Set to a world/environment USD to load that instead.
_DEFAULT_ENVIRONMENT_USD_PATH = ""

DEFAULTS = {
    "robot_usd_path": _DEFAULT_ROBOT_USD_PATH,
    "environment_usd_path": _DEFAULT_ENVIRONMENT_USD_PATH,
    "policy_path": _DEFAULT_POLICY_PATH,
    "policy_env_path": _DEFAULT_POLICY_ENV_PATH,
    "policy_deploy_path": _DEFAULT_POLICY_DEPLOY_PATH,
    # ROS2 bridge (disabled by default -- enabling it loads isaacsim.ros2.bridge
    # and builds a cmd_vel-in / odom+tf+joint_states+clock-out OmniGraph, see
    # ros2_bridge.py).
    "ros2_enabled": "False",
    "ros2_publish_clock": "True",
    "ros2_namespace": "",
    "ros2_domain_id": "",
    "ros2_chassis_frame": "base",
    "ros2_cmd_vel_topic": "cmd_vel",
    "ros2_odom_topic": "odom",
    "ros2_joint_states_topic": "joint_states",
    "ros2_tf_topic": "tf",
    # Mid-360 lidar (disabled by default -- mounts a Livox Mid-360-approximate
    # RTX Lidar on the robot's head when enabled, and publishes it to ROS2 as
    # a PointCloud2 if the ROS2 Bridge above is also enabled; see mid360.py).
    "mid360_enabled": "False",
    "mid360_translate": "0.28, 0.0, 0.10",
    "mid360_tilt_deg": "0.0",
    "mid360_topic": "livox/lidar",
    "mid360_frame_id": "livox_frame",
}

# Order + display metadata for the settings window.
FIELDS = [
    ("robot_usd_path", "Robot USD", "The Go2 (or Go2 + sensors) USD file to spawn.", [("USD files", "*.usd*")]),
    (
        "environment_usd_path",
        "Environment USD",
        "Optional world/environment USD to load instead of the default ground plane. Leave empty for the default ground plane.",
        [("USD files", "*.usd*")],
    ),
    ("policy_path", "Policy (.pt)", "TorchScript policy checkpoint, e.g. unitree_rl_lab's exported/policy.pt.", [("TorchScript policy", "*.pt")]),
    (
        "policy_env_path",
        "Policy env.yaml",
        "The Isaac Lab params/env.yaml dumped alongside the checkpoint (defines joint gains/defaults).",
        [("YAML", "*.yaml")],
    ),
    (
        "policy_deploy_path",
        "Policy deploy.yaml",
        "unitree_rl_lab's params/deploy.yaml dumped alongside the checkpoint. Defines the observation term"
        " list/order/scales and the action scale/offset, so the observation vector is built to match this"
        " checkpoint automatically instead of being hardcoded.",
        [("YAML", "*.yaml")],
    ),
]

# ROS2 bridge settings window: a single enable checkbox plus the topic/frame
# names nav2 (or any other ROS2 client) needs to match. Left blank/False by
# default so this extension behaves exactly as before unless turned on.
ROS2_ENABLE_FIELD = ("ros2_enabled", "Enable ROS2 Bridge", "Subscribe to cmd_vel and publish odom/tf/joint_states/clock.")

ROS2_TOGGLE_FIELDS = [
    (
        "ros2_publish_clock",
        "Publish /clock",
        "Publish simulation time on /clock. Needed if downstream ROS2 nodes (e.g. nav2) run with use_sim_time.",
    ),
]

ROS2_TEXT_FIELDS = [
    ("ros2_namespace", "Node Namespace", "Prefix applied to every topic below (leave empty for none)."),
    ("ros2_domain_id", "Domain ID", "Leave empty to use ROS_DOMAIN_ID from the environment (or 0)."),
    ("ros2_chassis_frame", "Chassis Frame Id", "TF/odometry frame id for the robot base (matches the USD's base link name)."),
    ("ros2_cmd_vel_topic", "cmd_vel Topic", "geometry_msgs/Twist topic that drives the robot (e.g. from nav2 or teleop)."),
    ("ros2_odom_topic", "Odometry Topic", "nav_msgs/Odometry topic published from the ground-truth chassis pose."),
    ("ros2_joint_states_topic", "Joint States Topic", "sensor_msgs/JointState topic published for the articulation."),
    ("ros2_tf_topic", "TF Topic", "Topic the world->odom->chassis and robot link transforms are published on."),
]

# Mid-360 lidar settings window.
MID360_ENABLE_FIELD = (
    "mid360_enabled",
    "Mount Mid-360 Lidar",
    "Mounts a Livox Mid-360-approximate RTX Lidar on the robot's head (works with any Robot USD --"
    " no sensor-equipped USD variant needed). Range/FOV approximate the real Mid-360's spec"
    " (360deg x -7..+52deg, ~40m); the exact non-repetitive scan pattern is not reproduced.",
)

MID360_TEXT_FIELDS = [
    (
        "mid360_translate",
        "Mount Offset (x,y,z m)",
        "Lidar position relative to the robot's articulation root, in meters (comma-separated). Adjust to match"
        " your actual Mid-360 mount bracket -- the shipped default is an approximate head-top placement.",
    ),
    (
        "mid360_tilt_deg",
        "Mount Tilt (deg)",
        "Pitch tilt applied on top of a level mount, in degrees. Sign/axis is a best-effort convention --"
        " check the point cloud in RViz and flip the sign if it tilts the wrong way.",
    ),
    ("mid360_topic", "PointCloud2 Topic", "Topic the Mid-360's PointCloud2 is published on (only if ROS2 Bridge is also enabled)."),
    ("mid360_frame_id", "Frame Id", "TF frame id for the lidar (published as a static child of the chassis frame)."),
]


def get(key: str) -> str:
    value = carb.settings.get_settings().get(f"{_ROOT}/{key}")
    if value:
        return value
    return DEFAULTS[key]


def set(key: str, value: str) -> None:
    carb.settings.get_settings().set(f"{_ROOT}/{key}", value)


def reset(key: str) -> None:
    carb.settings.get_settings().set(f"{_ROOT}/{key}", DEFAULTS[key])
