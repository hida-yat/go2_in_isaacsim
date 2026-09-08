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


def find_arm(robot_prim_path: str, mount_name: str = "Piper"):
    """Looks for the Piper's own articulation root, scoped to
    {robot_prim_path}/{mount_name} -- not a whole-robot schema scan like
    mid360.find_sensor(), because the arm is *itself* a second, independent
    PhysX articulation (its own ArticulationRootAPI, distinct from the
    chassis's) welded to the chassis by a fixed joint, and an unscoped scan
    could just as easily return the chassis's own articulation root. The
    mount is a *sibling* of the chassis's `base` prim, not its child --
    `base` carries Go2's own ArticulationRootAPI, and PhysX forbids nesting
    one articulation root under another rigid body that's already part of
    an articulation (see tools/build_go2_with_mid360_and_piper.py). Returns
    None if the loaded Robot USD has no Piper mounted there (e.g. bare
    go2.usd or go2_with_mid360.usd)."""
    stage = omni.usd.get_context().get_stage()
    mount_prim = stage.GetPrimAtPath(f"{robot_prim_path}/{mount_name}")
    if not mount_prim.IsValid():
        return None
    for prim in Usd.PrimRange(mount_prim):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return prim
    return None


def publish_to_ros2(arm_prim) -> None:
    """Builds the OmniGraph publishing arm_prim's joint_states and driving
    it from joint_command (position/velocity/effort arrays, by joint name --
    what a FollowJointTrajectory-to-topic bridge on the ROS2/MoveIt side
    would publish)."""
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(GRAPH_PATH).IsValid():
        stage.RemovePrim(GRAPH_PATH)

    node_namespace = settings.get("ros2_namespace")
    domain_id = settings.get("ros2_domain_id")
    arm_prim_path = arm_prim.GetPath().pathString

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.SET_VALUES: [
                ("ReadSimTime.inputs:resetOnStop", False),
                ("PublishJointState.inputs:targetPrim", arm_prim_path),
                ("PublishJointState.inputs:topicName", settings.get("piper_joint_states_topic")),
                ("PublishJointState.inputs:nodeNamespace", node_namespace),
                ("SubscribeJointState.inputs:topicName", settings.get("piper_joint_command_topic")),
                ("SubscribeJointState.inputs:nodeNamespace", node_namespace),
                ("ArticulationController.inputs:targetPrim", arm_prim_path),
            ],
            keys.CONNECT: [
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
            ],
        },
    )

    if domain_id:
        og.Controller.attribute(f"{GRAPH_PATH}/Context.inputs:domain_id").set(int(domain_id))
        og.Controller.attribute(f"{GRAPH_PATH}/Context.inputs:useDomainIDEnvVar").set(False)
