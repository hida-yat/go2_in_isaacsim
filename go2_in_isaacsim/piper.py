# Publishes the Piper arm's joint_states/joint_command over ROS2, if the
# loaded Robot USD has one mounted (see ROBOT_PRESETS / build with
# tools/build_go2_with_mid360_and_piper.py). Mirrors mid360.py/imu.py's
# shape: find_* locates the thing on the currently loaded robot,
# publish_to_ros2 wires its OmniGraph, both called from go2_example.py.
#
# Unlike the lidar/IMU, the arm needs a *subscriber* too -- there's no
# onboard policy driving it, so cmd_vel-style "keyboard overrides ROS2" does
# not apply here: whatever's on joint_command drives it, always. Publisher +
# subscriber + IsaacArticulationController is the same OG node combination
# isaacsim.ros2.bridge's own "Extensions > ROS2 > Joint States" graph
# shortcut uses (og_shortcuts/og_utils.py's Ros2JointStatesGraph).

import omni.graph.core as og
import omni.usd
from pxr import Usd, UsdPhysics

from . import settings

GRAPH_PATH = "/World/Go2PiperROS2"

# The bundled Piper USD's own auto-authored gripper (joint7/joint8, prismatic
# finger joints) drive gains are ~150x weaker than the arm's per their own
# units (stiffness ~0.19 N/m vs the arm's 8-60 N*m/rad) -- almost certainly an
# artifact of whatever auto-computed them from the fingers' tiny mass/inertia
# at import time. maxForce is already a reasonable 100N; only stiffness/
# damping need boosting, or the gripper just sits there with ~zero effective
# holding/actuating torque no matter what position is commanded.
_GRIPPER_JOINT_NAMES = ("joint7", "joint8")
_GRIPPER_DRIVE_STIFFNESS = 2000.0  # N/m
_GRIPPER_DRIVE_DAMPING = 20.0  # N*s/m


def find_arm(robot_prim_path: str, mount_name: str = "Piper"):
    """Looks for the Piper's own articulation root, scoped to *the parent*
    of robot_prim_path -- not a whole-robot schema scan like
    mid360.find_sensor(), because the arm is *itself* a second, independent
    PhysX articulation (its own ArticulationRootAPI, distinct from the
    chassis's) welded to the chassis by a fixed joint, and an unscoped scan
    could just as easily return the chassis's own articulation root.

    robot_prim_path is expected to be go2_example.py's self.go2.robot.prim_path
    -- isaacsim.core.prims.SingleArticulation resolves that to the prim that
    actually carries PhysicsArticulationRootAPI, i.e. Go2's `base` prim
    (.../base), *not* the robot's outer Xform (.../Go2) callers might expect.
    The Piper mount is a *sibling* of `base`, not its child -- `base`
    already carries Go2's own ArticulationRootAPI, and PhysX forbids nesting
    one articulation root under another rigid body that's already part of
    an articulation (see tools/build_go2_with_mid360_and_piper.py) -- so it
    has to be looked up via base's *parent*, not a subtree scan of base
    itself (which is exactly why this was silently returning None: an
    earlier version scoped the lookup to {robot_prim_path}/{mount_name},
    i.e. .../base/Piper, which never existed). Returns None if the loaded
    Robot USD has no Piper mounted there (e.g. bare go2.usd or
    go2_with_mid360.usd)."""
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        return None
    parent_prim = robot_prim.GetParent()
    if not parent_prim.IsValid():
        return None
    mount_prim = stage.GetPrimAtPath(parent_prim.GetPath().AppendChild(mount_name))
    if not mount_prim.IsValid():
        return None
    for prim in Usd.PrimRange(mount_prim):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return prim
    return None


def _boost_gripper_drive_gains(arm_prim) -> None:
    """Raises joint7/joint8's PhysicsDriveAPI stiffness/damping to
    _GRIPPER_DRIVE_STIFFNESS/_GRIPPER_DRIVE_DAMPING (see module docstring
    above). Scoped to arm_prim's parent (the Piper mount Xform) rather than
    arm_prim's own subtree -- arm_prim is root_joint itself (see find_arm's
    docstring: ArticulationRootAPI lands on the fixed weld joint in this USD,
    not a body prim), whose own USD subtree contains no joints at all; the
    actual joint1..joint8 prims live elsewhere under the mount, findable by
    PhysX/Isaac's articulation APIs but not by a naive PrimRange from
    arm_prim itself."""
    mount_prim = arm_prim.GetParent()
    for prim in Usd.PrimRange(mount_prim):
        if prim.GetName() in _GRIPPER_JOINT_NAMES and prim.HasAPI(UsdPhysics.DriveAPI, "linear"):
            drive = UsdPhysics.DriveAPI(prim, "linear")
            drive.GetStiffnessAttr().Set(_GRIPPER_DRIVE_STIFFNESS)
            drive.GetDampingAttr().Set(_GRIPPER_DRIVE_DAMPING)


def publish_to_ros2(arm_prim, robot_prim_path: str) -> None:
    """Builds the OmniGraph publishing arm_prim's joint_states and driving
    it from joint_command (position/velocity/effort arrays, by joint name --
    what a FollowJointTrajectory-to-topic bridge on the ROS2/MoveIt side
    would publish), plus -- only if the "piper_publish_arm_tf" Preference is
    True -- the arm's own link TF tree (link1..link6 etc.), relative to
    robot_prim_path (Go2's base -- same reference prim ros2_bridge.py's own
    chassis TF uses, so this joins the same tree at chassis_frame). Unlike
    the Mid-360's *rigid* chassis->lidar mount (mid360.py's
    _relative_transform, computed once), the arm actually moves, so this
    has to be ROS2PublishTransformTree's live per-tick articulation walk,
    not a one-shot static offset.

    Leave that Preference off when a real Piper URDF's robot_state_publisher
    is *also* running against this same arm (piper_isaacsim_bringup.
    launch.py's MoveIt stack, via topic_based_ros2_control reading this
    node's own joint_states topic) -- it publishes the identical
    link1..link6 names from real joint_states, rooted at its own URDF's
    "world"/"base_link", and having *this* publish the same names too (from
    Isaac's own simulation-truth, rooted at Go2's unrelated "world") makes
    tf2 flicker between the two on every lookup (see settings.py's
    "piper_publish_arm_tf" comment). realsense.py's own camera TF edge is
    parented at "link6" either way, so it keeps working off of whichever
    one is actually publishing that tree."""
    _boost_gripper_drive_gains(arm_prim)

    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(GRAPH_PATH).IsValid():
        stage.RemovePrim(GRAPH_PATH)

    node_namespace = settings.get("ros2_namespace")
    domain_id = settings.get("ros2_domain_id")
    arm_prim_path = arm_prim.GetPath().pathString
    publish_tf = settings.get("piper_publish_arm_tf") == "True"

    keys = og.Controller.Keys
    create_nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
        ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
    ]
    set_values = [
        ("ReadSimTime.inputs:resetOnStop", False),
        ("PublishJointState.inputs:targetPrim", arm_prim_path),
        ("PublishJointState.inputs:topicName", settings.get("piper_joint_states_topic")),
        ("PublishJointState.inputs:nodeNamespace", node_namespace),
        ("SubscribeJointState.inputs:topicName", settings.get("piper_joint_command_topic")),
        ("SubscribeJointState.inputs:nodeNamespace", node_namespace),
        ("ArticulationController.inputs:targetPrim", arm_prim_path),
    ]
    connect = [
        ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
        ("Context.outputs:context", "PublishJointState.inputs:context"),
        ("Context.outputs:context", "SubscribeJointState.inputs:context"),
        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
        ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
        ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
        ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
        ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
    ]

    if publish_tf:
        create_nodes.append(("PublishArmTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"))
        set_values += [
            ("PublishArmTF.inputs:parentPrim", robot_prim_path),
            ("PublishArmTF.inputs:targetPrims", arm_prim_path),
            ("PublishArmTF.inputs:topicName", settings.get("ros2_tf_topic")),
            ("PublishArmTF.inputs:nodeNamespace", node_namespace),
        ]
        connect += [
            ("OnPlaybackTick.outputs:tick", "PublishArmTF.inputs:execIn"),
            ("Context.outputs:context", "PublishArmTF.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublishArmTF.inputs:timeStamp"),
        ]

    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {keys.CREATE_NODES: create_nodes, keys.SET_VALUES: set_values, keys.CONNECT: connect},
    )

    if domain_id:
        og.Controller.attribute(f"{GRAPH_PATH}/Context.inputs:domain_id").set(int(domain_id))
        og.Controller.attribute(f"{GRAPH_PATH}/Context.inputs:useDomainIDEnvVar").set(False)


# piper_ros's actual real-hardware driver (the "piper" package's
# piper_ctrl_single_node.py -- confirmed by checking setup.py's console_scripts,
# not the unused piper_ctrl_single_node_new.py) exposes a 7-element JointState
# on both its state and command topics: name=['joint1'..'joint6','gripper'],
# where the gripper is *one* combined value (real hardware has no joint7/
# joint8 split) read by piper_ctrl_single_node.py's joint_callback() strictly
# by *array index* (position[6], not by name) -- see
# ~/devel/ros2/workspaces/piper_ros/src/piper/piper/piper_ctrl_single_node.py.
_HW_ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
_HW_GRIPPER_DRIVE_JOINT = "joint7"
_HW_GRIPPER_MIRROR_JOINT = "joint8"


class HardwareCompatibleBridge:
    """A *separate*, additive raw ROS2 interface from publish_to_ros2's own
    piper/joint_states + piper/joint_command (8-joint, joint1..joint8 --
    used by piper_isaacsim's topic_based_ros2_control for MoveIt): this one
    mirrors piper_ctrl_single_node.py's own topics/shape exactly, so ROS2
    code written against real Piper hardware (e.g. piper_ros's own
    joy_to_piper_joint_states) runs against Isaac Sim unmodified. Nothing
    stops both interfaces being wired up at once (real hardware users don't
    run piper_single_ctrl and Gazebo/MoveIt against the same arm
    simultaneously either) -- just don't send commands on both at once, or
    whichever's ArticulationController write lands last each tick wins.

    Uses a plain rclpy Node instead of OmniGraph -- this codebase's only one.
    Neither ROS2PublishJointState nor ROS2SubscribeJointState can remap or
    combine joint names (ROS2PublishJointState in particular has no data
    inputs at all: it always derives name/position/velocity/effort straight
    from targetPrim's own PhysX DOF names, so intercepting/rewriting its
    output isn't possible either), so the gripper-combining logic below has
    to happen in plain Python somewhere.

    Domain ID: shares whichever *global default* rclpy context
    isaacsim.ros2.bridge itself already initialized (plain ROS_DOMAIN_ID from
    the environment) -- unlike this extension's OmniGraph pipelines (which
    each get their own ROS2Context node), this does NOT honor an explicit
    non-empty "ros2_domain_id" Preferences override. Fine for the common case
    (that field left empty); revisit if a multi-domain Piper setup needs it.
    """

    def __init__(self, robot, states_topic: str, command_topic: str, namespace: str) -> None:
        import rclpy
        from sensor_msgs.msg import JointState

        self._robot = robot
        self._arm_indices = {j: robot.dof_names.index(j) for j in _HW_ARM_JOINTS}
        self._gripper_drive_index = robot.dof_names.index(_HW_GRIPPER_DRIVE_JOINT)
        self._gripper_mirror_index = robot.dof_names.index(_HW_GRIPPER_MIRROR_JOINT)
        self._JointState = JointState
        self._rclpy = rclpy

        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node("piper_hardware_compatible_bridge", namespace=namespace or None)
        self._pub = self._node.create_publisher(JointState, states_topic, 1)
        self._latest_command = None
        self._node.create_subscription(JointState, command_topic, self._on_command, 1)

    def _on_command(self, msg) -> None:
        self._latest_command = msg

    def step(self) -> None:
        """Call once per physics tick: drains any pending joint_command
        message (applying it to the articulation) and publishes fresh
        joint_states_single feedback, both in piper_ctrl_single_node.py's own
        7-element shape."""
        from isaacsim.core.utils.types import ArticulationAction

        self._rclpy.spin_once(self._node, timeout_sec=0.0)

        cmd = self._latest_command
        if cmd is not None:
            self._latest_command = None
            name_to_position = dict(zip(cmd.name, cmd.position))
            positions, indices = [], []
            for joint_name, index in self._arm_indices.items():
                if joint_name in name_to_position:
                    positions.append(name_to_position[joint_name])
                    indices.append(index)
            # Matches joint_callback()'s own gripper handling exactly: read by
            # *array index* (position[6]), independent of what (if anything)
            # cmd.name[6] says -- mirrored onto both gripper fingers since
            # Isaac's PhysX joints aren't set up with a true mimic constraint
            # (see _boost_gripper_drive_gains's docstring above).
            if len(cmd.position) >= 7:
                positions += [cmd.position[6], -cmd.position[6]]
                indices += [self._gripper_drive_index, self._gripper_mirror_index]
            if indices:
                self._robot.apply_action(ArticulationAction(joint_positions=positions, joint_indices=indices))

        read_indices = list(self._arm_indices.values()) + [self._gripper_drive_index]
        current_positions = self._robot.get_joint_positions(joint_indices=read_indices)

        msg = self._JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = list(_HW_ARM_JOINTS) + ["gripper"]
        msg.position = [float(p) for p in current_positions]
        self._pub.publish(msg)

    def shutdown(self) -> None:
        self._node.destroy_node()


def build_hardware_compatible_bridge(robot) -> HardwareCompatibleBridge:
    """robot is go2_example.py's self.go2.robot (isaacsim.core.prims.
    SingleArticulation) -- unlike find_arm/publish_to_ros2 above, this needs
    a live, *initialized* articulation handle (robot.dof_names,
    robot.apply_action) to read/drive joint positions directly in Python, not
    just a USD prim path, so call this only after go2.initialize() (i.e. from
    go2_example.py's on_physics_step first-tick branch, not setup_scene)."""
    return HardwareCompatibleBridge(
        robot,
        settings.get("piper_hw_joint_states_topic"),
        settings.get("piper_hw_joint_command_topic"),
        settings.get("ros2_namespace"),
    )
