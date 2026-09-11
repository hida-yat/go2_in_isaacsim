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
    # Mid-360 lidar: purely a publish-topic/frame naming choice. Whether
    # there IS a Mid-360 to publish depends on which Robot USD is loaded
    # (see ROBOT_PRESETS / go2_with_mid360.usd) -- see mid360.py.
    "mid360_topic": "livox/lidar",
    "mid360_frame_id": "livox_frame",
    "mid360_imu_topic": "livox/imu",
    "mid360_imu_frame_id": "livox_imu_frame",
    # Piper arm: like Mid-360, purely a topic naming choice -- whether one is
    # actually there depends on the loaded Robot USD (go2_with_mid360_and_piper.usd)
    # -- see piper.py. Distinct from ros2_joint_states_topic ("joint_states"):
    # the arm is a second, independent articulation, not part of the chassis's.
    "piper_joint_states_topic": "piper/joint_states",
    "piper_joint_command_topic": "piper/joint_command",
    # Whether piper.py publishes the arm's own link1..link6 (+camera_link)
    # TF tree. Leave True for a bare Isaac Sim + RViz session with no other
    # ROS2 nodes running (nothing else would ever publish this tree
    # otherwise). Turn OFF if a real Piper URDF-based robot_state_publisher
    # is *also* running against the same arm (e.g. piper_isaacsim_bringup.
    # launch.py's MoveIt stack, via topic_based_ros2_control) -- that one
    # already publishes link1..link6 from real joint_states, rooted at its
    # own URDF's "world"/"base_link"; leaving this on too means TWO
    # publishers claim the same link names from two unrelated "world"
    # roots (Isaac's own vs. the URDF's), which tf2 resolves by whichever
    # arrives last -- symptom: a PointCloud2/OctoMap that flickers between
    # correct and wildly-wrong placement every frame. realsense.py's own
    # camera TF edge is parented at "link6" either way, so it keeps working
    # once the real robot_state_publisher is the sole source.
    "piper_publish_arm_tf": "True",
    # Separate, additive raw interface matching piper_ros's actual real-hardware
    # driver (piper_ctrl_single_node.py) exactly -- 7-element JointState with a
    # single combined "gripper" DOF instead of joint7/joint8 separately -- so
    # ROS2 code written against real Piper hardware runs against this
    # unmodified. See piper.py's HardwareCompatibleBridge.
    "piper_hw_joint_states_topic": "joint_states_single",
    "piper_hw_joint_command_topic": "joint_command",
    # Piper's wrist-mounted D435 RealSense: purely decorative mesh in the
    # bundled Piper USD (no Camera prim at all) until realsense.py creates
    # one -- see its module docstring. Topic naming loosely follows
    # realsense2_camera's own convention (color/, depth/); RGB and Depth
    # share one render product (same simulated camera prim) so they're
    # inherently pixel-aligned already and one camera_info covers both.
    "realsense_rgb_topic": "realsense/color/image_raw",
    "realsense_depth_topic": "realsense/depth/image_rect_raw",
    "realsense_camera_info_topic": "realsense/color/camera_info",
    # realsense2_camera publishes this as depth/color/points (registered
    # depth+color point cloud) when pointcloud.enable:=true.
    "realsense_pointcloud_topic": "realsense/depth/color/points",
    "realsense_frame_id": "d435_color_optical_frame",
    "realsense_width": "640",
    "realsense_height": "480",
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

# Quick-pick presets for "environment_usd_path", shown as a dropdown above
# that field in Preferences. Values starting with "/Isaac/" are resolved
# against Isaac Sim's Nucleus assets root at Load time (see go2_example.py);
# "Custom..." leaves the text field alone so a manually typed/browsed path
# is never overwritten. These are Isaac Sim's own bundled sample
# environments (isaacsim/standalone_examples reference the same paths), not
# assets authored by this repo.
ENVIRONMENT_PRESETS = [
    ("Default Ground Plane", ""),
    ("Grid - Default", "/Isaac/Environments/Grid/default_environment.usd"),
    ("Grid Room (Black)", "/Isaac/Environments/Grid/gridroom_black.usd"),
    ("Simple Room", "/Isaac/Environments/Simple_Room/simple_room.usd"),
    ("Warehouse", "/Isaac/Environments/Simple_Warehouse/warehouse.usd"),
    ("Warehouse (Full)", "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"),
    ("Warehouse with Forklifts", "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"),
    ("Custom...", None),
]

# Same quick-pick pattern as ENVIRONMENT_PRESETS, for "robot_usd_path".
# These are bundled local files (unlike the Nucleus-relative environment
# presets), so the paths are resolved immediately, not at Load time.
ROBOT_PRESETS = [
    ("Go2 (bare)", _DEFAULT_ROBOT_USD_PATH),
    ("Go2 with Mid-360", os.path.join(_EXT_ROOT, "data", "Robots", "Go2", "usd", "go2_with_mid360.usd")),
    ("Go2 with Mid-360 + Piper", os.path.join(_EXT_ROOT, "data", "Robots", "Go2", "usd", "go2_with_mid360_and_piper.usd")),
    ("Custom...", None),
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

# Mid-360 lidar settings window. Whether one gets published at all depends
# on whether the loaded Robot USD has one (see ROBOT_PRESETS above) -- these
# fields name the lidar and its runtime physics IMU.
MID360_TEXT_FIELDS = [
    ("mid360_topic", "PointCloud2 Topic", "Topic the Mid-360's PointCloud2 is published on, if the loaded Robot USD has one (only if ROS2 Bridge is also enabled)."),
    ("mid360_frame_id", "Frame Id", "TF frame id for the lidar (published as a child of the chassis frame, at its actual mount transform)."),
    ("mid360_imu_topic", "IMU Topic", "sensor_msgs/Imu at the physics rate (200 Hz); acceleration in m/s^2 including gravity and angular velocity in rad/s."),
    ("mid360_imu_frame_id", "IMU Frame Id", "Separate mount-aligned IMU frame, connected to the chassis by TF. Must differ from the lidar and chassis frame IDs."),
]

# Piper arm settings window. Whether one gets published at all depends on
# whether the loaded Robot USD has one (see ROBOT_PRESETS above).
PIPER_TOGGLE_FIELDS = [
    (
        "piper_publish_arm_tf",
        "Publish Arm TF",
        "Turn OFF if a real Piper URDF's robot_state_publisher is also running against this same arm "
        "(e.g. piper_isaacsim_bringup.launch.py's MoveIt stack) -- otherwise two unrelated TF trees both "
        "claim link1..link6, which tf2 resolves by last-writer-wins and flickers between them.",
    ),
]

PIPER_TEXT_FIELDS = [
    ("piper_joint_states_topic", "Joint States Topic", "sensor_msgs/JointState telemetry for the arm, if the loaded Robot USD has one."),
    ("piper_joint_command_topic", "Joint Command Topic", "sensor_msgs/JointState (position/velocity/effort by joint name) that drives the arm -- e.g. from a FollowJointTrajectory-to-topic bridge on the MoveIt side."),
    (
        "piper_hw_joint_states_topic",
        "Hardware-Compatible Joint States Topic",
        "sensor_msgs/JointState telemetry matching piper_ros's real-hardware driver exactly (joint1..6 + one combined 'gripper' DOF), separate from Joint States Topic above.",
    ),
    (
        "piper_hw_joint_command_topic",
        "Hardware-Compatible Joint Command Topic",
        "sensor_msgs/JointState command matching piper_ros's real-hardware driver exactly (position[6] is a single combined gripper value, mirrored internally to both gripper fingers).",
    ),
]

# Piper's wrist-mounted D435 RealSense settings window. Whether one gets
# published at all depends on whether the loaded Robot USD has a Piper with
# that camera mesh mounted (see ROBOT_PRESETS above) -- see realsense.py.
REALSENSE_TEXT_FIELDS = [
    ("realsense_rgb_topic", "RGB Topic", "sensor_msgs/Image (rgb8) from the D435's color stream."),
    ("realsense_depth_topic", "Depth Topic", "sensor_msgs/Image (32FC1, meters) from the D435's depth stream. Pixel-aligned with RGB (same simulated camera)."),
    ("realsense_camera_info_topic", "Camera Info Topic", "sensor_msgs/CameraInfo, shared by both RGB and Depth (same render product, so the same intrinsics apply to both)."),
    ("realsense_pointcloud_topic", "Point Cloud Topic", "sensor_msgs/PointCloud2, colorized and reprojected from the same Depth stream (matches realsense2_camera's depth/color/points)."),
    ("realsense_frame_id", "Frame Id", "TF frame id for the camera optical frame, in image headers."),
    ("realsense_width", "Width (px)", "Rendered image width."),
    ("realsense_height", "Height (px)", "Rendered image height."),
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
