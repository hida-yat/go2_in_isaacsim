# Bridges this example's Go2 to ROS2: subscribes to cmd_vel (geometry_msgs/Twist)
# so an external teleop or nav2 can drive the robot instead of (or alongside)
# the keyboard, and publishes the topics nav2 expects from a mobile base --
# odometry, the odom/tf tree, joint_states, and (optionally) /clock.
#
# The graph mirrors the vendor shortcuts shipped in isaacsim.ros2.bridge's
# own "Extensions > ROS2 > ..." menu builders (see
# isaacsim/ros2/bridge/impl/og_shortcuts/og_utils.py -- Ros2OdometryGraph,
# Ros2JointStatesGraph, Ros2ClockGraph), just assembled from Python once at
# Load instead of by hand from that menu, and using the robot's articulation
# root (always valid, regardless of how a swapped-in go2.usd names its
# internal links) as the odometry/TF reference prim instead of a specific
# child link.

import numpy as np
import omni.graph.core as og
import omni.usd

from . import settings

GRAPH_PATH = "/World/Go2ROS2"
SUBSCRIBE_TWIST_NODE = "SubscribeCmdVel"


def is_available() -> bool:
    """Tries to enable isaacsim.ros2.bridge, returning False if that fails
    (e.g. no compatible ROS2 environment/libraries found)."""
    try:
        import omni.kit.app
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.ros2.bridge")
        return omni.kit.app.get_app().get_extension_manager().is_extension_enabled("isaacsim.ros2.bridge")
    except Exception:
        return False


def build_graph(robot_prim_path: str) -> str:
    """(Re)builds the ROS2 bridge OmniGraph for the robot at robot_prim_path.

    Returns the prim path of the ROS2SubscribeTwist node so the caller can
    poll its latest cmd_vel value each physics step via read_cmd_vel().
    """
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(GRAPH_PATH).IsValid():
        stage.RemovePrim(GRAPH_PATH)

    node_namespace = settings.get("ros2_namespace")
    chassis_frame = settings.get("ros2_chassis_frame")
    domain_id = settings.get("ros2_domain_id")

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                (SUBSCRIBE_TWIST_NODE, "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("TFWorldToOdom", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ("TFOdomToChassis", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ("PublishRobotTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.SET_VALUES: [
                ("ReadSimTime.inputs:resetOnStop", False),
                (f"{SUBSCRIBE_TWIST_NODE}.inputs:topicName", settings.get("ros2_cmd_vel_topic")),
                (f"{SUBSCRIBE_TWIST_NODE}.inputs:nodeNamespace", node_namespace),
                ("ComputeOdometry.inputs:chassisPrim", robot_prim_path),
                ("PublishOdometry.inputs:topicName", settings.get("ros2_odom_topic")),
                ("PublishOdometry.inputs:chassisFrameId", chassis_frame),
                ("PublishOdometry.inputs:nodeNamespace", node_namespace),
                ("TFWorldToOdom.inputs:parentFrameId", "world"),
                ("TFWorldToOdom.inputs:childFrameId", "odom"),
                ("TFWorldToOdom.inputs:nodeNamespace", node_namespace),
                ("TFOdomToChassis.inputs:parentFrameId", "odom"),
                ("TFOdomToChassis.inputs:childFrameId", chassis_frame),
                ("TFOdomToChassis.inputs:nodeNamespace", node_namespace),
                ("PublishRobotTF.inputs:parentPrim", robot_prim_path),
                ("PublishRobotTF.inputs:targetPrims", robot_prim_path),
                ("PublishRobotTF.inputs:topicName", settings.get("ros2_tf_topic")),
                ("PublishRobotTF.inputs:nodeNamespace", node_namespace),
                ("PublishJointState.inputs:targetPrim", robot_prim_path),
                ("PublishJointState.inputs:topicName", settings.get("ros2_joint_states_topic")),
                ("PublishJointState.inputs:nodeNamespace", node_namespace),
                ("PublishClock.inputs:topicName", "/clock"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", f"{SUBSCRIBE_TWIST_NODE}.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "TFWorldToOdom.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "TFOdomToChassis.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishRobotTF.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", f"{SUBSCRIBE_TWIST_NODE}.inputs:context"),
                ("Context.outputs:context", "PublishOdometry.inputs:context"),
                ("Context.outputs:context", "TFWorldToOdom.inputs:context"),
                ("Context.outputs:context", "TFOdomToChassis.inputs:context"),
                ("Context.outputs:context", "PublishRobotTF.inputs:context"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "TFWorldToOdom.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "TFOdomToChassis.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishRobotTF.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
                ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
                ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
                ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
                ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
                ("ComputeOdometry.outputs:orientation", "TFOdomToChassis.inputs:rotation"),
                ("ComputeOdometry.outputs:position", "TFOdomToChassis.inputs:translation"),
            ],
        },
    )

    if domain_id:
        og.Controller.attribute(f"{GRAPH_PATH}/Context.inputs:domain_id").set(int(domain_id))
        og.Controller.attribute(f"{GRAPH_PATH}/Context.inputs:useDomainIDEnvVar").set(False)

    if settings.get("ros2_publish_clock") != "True":
        stage.RemovePrim(f"{GRAPH_PATH}/PublishClock")

    return f"{GRAPH_PATH}/{SUBSCRIBE_TWIST_NODE}"


def read_cmd_vel(subscribe_node_path: str) -> np.ndarray:
    """Reads the latest [vx, vy, wz] received on cmd_vel (0s if nothing has
    arrived yet -- ROS2SubscribeTwist holds its last received value between
    messages, same latch-until-updated behavior as a real cmd_vel consumer)."""
    linear = og.Controller.attribute(f"{subscribe_node_path}.outputs:linearVelocity").get()
    angular = og.Controller.attribute(f"{subscribe_node_path}.outputs:angularVelocity").get()
    return np.array([linear[0], linear[1], angular[2]], dtype=np.float64)
