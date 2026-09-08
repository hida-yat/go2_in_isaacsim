# Publishes a Mid-360 lidar -- baked into a Robot USD such as the bundled
# go2_with_mid360.usd (see data/Robots/Go2/usd/go2_with_mid360.usd, built
# from go2.usd + data/Sensors/Mid360/Mid360.usd, a real Mid-360 CAD model
# converted to USD, with an RTX Lidar sensor API applied) -- as a ROS2
# PointCloud2, if one is present on the currently loaded robot.
#
# This does NOT mount a sensor at runtime: the sensor is part of the Robot
# USD itself (visual geometry + lidar API on the same prim tree, so what you
# see is what's actually sensing). go2.usd (bare) and go2_with_mid360.usd are
# separate files -- pick one via Edit > Preferences > Go2 Policy Example >
# Assets > Robot USD (or its Preset dropdown) -- rather than every Robot USD
# silently growing a lidar. find_sensor() below just looks for one.
#
# There is no official Mid-360 profile bundled with Isaac Sim's RTX Lidar
# (isaacsim.sensors.rtx/data/lidar_configs/ has Velodyne/Ouster/Hesai/SICK/...
# but no Livox), so this ships a custom profile (data/lidar_configs/Livox/
# Mid360.json, "rotary" scanType) matching the real sensor's headline specs
# -- 360deg horizontal x -7..+52deg vertical FOV, ~40m range, ~200,000
# points/sec, 905nm -- using 40 evenly-spaced vertical channels swept through
# a full rotation. This is an envelope match, not a reproduction of the real
# Mid-360's non-repetitive (rosette) scan pattern.

import os

import carb.settings
import omni.graph.core as og
import omni.usd
from pxr import Usd, UsdGeom

from . import settings

_CONFIG_NAME = "Mid360"
_CONFIG_DIR = os.path.join(settings.EXT_ROOT, "data", "lidar_configs", "Livox")
_PROFILE_SEARCH_PATH_SETTING = "/app/sensors/nv/lidar/profileBaseFolder"
_LIDAR_API = "IsaacRtxLidarSensorAPI"

GRAPH_PATH = "/World/Go2Mid360ROS2"


def _ensure_profile_search_path() -> None:
    """Adds this extension's data/lidar_configs/Livox/ to the RTX lidar
    plugin's profile search path (additive -- preserves Isaac Sim's own
    vendor config directories already registered there). Only matters if the
    loaded Robot USD actually has a Mid-360 sensor using this config name."""
    s = carb.settings.get_settings()
    current = list(s.get(_PROFILE_SEARCH_PATH_SETTING) or [])
    if _CONFIG_DIR not in current:
        current.append(_CONFIG_DIR)
        s.set(_PROFILE_SEARCH_PATH_SETTING, current)


def is_available() -> bool:
    try:
        import omni.kit.app
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.sensors.rtx")
        return omni.kit.app.get_app().get_extension_manager().is_extension_enabled("isaacsim.sensors.rtx")
    except Exception:
        return False


def find_sensor(robot_prim_path: str):
    """Looks for an RTX Lidar sensor (a prim with IsaacRtxLidarSensorAPI
    applied) anywhere under the loaded robot. Returns the first one found,
    or None if the loaded Robot USD doesn't have one (e.g. bare go2.usd)."""
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        return None
    for prim in Usd.PrimRange(robot_prim):
        if prim.HasAPI(_LIDAR_API):
            return prim
    return None


def _relative_transform(sensor_prim, reference_prim_path: str):
    """Sensor's translation/rotation relative to reference_prim_path (the
    robot's articulation root, same reference ros2_bridge.py uses for
    odometry/TF), computed from the actual USD hierarchy -- not a guessed
    offset -- so the published TF always matches wherever the sensor is
    really mounted, in any Robot USD."""
    stage = sensor_prim.GetStage()
    reference_prim = stage.GetPrimAtPath(reference_prim_path)
    sensor_to_world = UsdGeom.Xformable(sensor_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    reference_to_world = UsdGeom.Xformable(reference_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    sensor_to_reference = sensor_to_world * reference_to_world.GetInverse()
    translation = sensor_to_reference.ExtractTranslation()
    quat = sensor_to_reference.ExtractRotationQuat()
    imaginary = quat.GetImaginary()
    return translation, (quat.GetReal(), imaginary[0], imaginary[1], imaginary[2])


def publish_to_ros2(sensor_prim, robot_prim_path: str, chassis_frame: str) -> None:
    """Builds the OmniGraph that publishes sensor_prim (from find_sensor) as
    a ROS2 PointCloud2, plus its chassis->lidar TF (computed from the prim's
    actual transform -- see _relative_transform -- on the same tf topic as
    ros2_bridge.py's world->odom->chassis chain, so it shows up connected to
    the rest of the tree)."""
    _ensure_profile_search_path()

    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(GRAPH_PATH).IsValid():
        stage.RemovePrim(GRAPH_PATH)

    node_namespace = settings.get("ros2_namespace")
    domain_id = settings.get("ros2_domain_id")
    lidar_frame = settings.get("mid360_frame_id")
    sensor_prim_path = sensor_prim.GetPath().pathString
    translation, orientation = _relative_transform(sensor_prim, robot_prim_path)

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishPointCloud", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                ("TFChassisToLidar", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ],
            keys.SET_VALUES: [
                ("ReadSimTime.inputs:resetOnStop", False),
                ("CreateRenderProduct.inputs:cameraPrim", sensor_prim_path),
                ("CreateRenderProduct.inputs:width", 1),
                ("CreateRenderProduct.inputs:height", 1),
                ("PublishPointCloud.inputs:type", "point_cloud"),
                ("PublishPointCloud.inputs:topicName", settings.get("mid360_topic")),
                ("PublishPointCloud.inputs:frameId", lidar_frame),
                ("PublishPointCloud.inputs:nodeNamespace", node_namespace),
                ("TFChassisToLidar.inputs:parentFrameId", chassis_frame),
                ("TFChassisToLidar.inputs:childFrameId", lidar_frame),
                ("TFChassisToLidar.inputs:translation", translation),
                ("TFChassisToLidar.inputs:rotation", orientation),
                # Not staticPublisher=True: that publishes with different QoS
                # (tf2's static-transform convention) and, in practice here,
                # this transform then doesn't show up as connected to the
                # rest of the tree in RViz. Publishing every tick on the same
                # "tf" topic as ros2_bridge.py's own chain (world->odom->
                # chassis) -- exactly like that chain's own raw transforms --
                # keeps this one connected the same way.
                ("TFChassisToLidar.inputs:topicName", settings.get("ros2_tf_topic")),
                ("TFChassisToLidar.inputs:nodeNamespace", node_namespace),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "PublishPointCloud.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "PublishPointCloud.inputs:renderProductPath"),
                ("Context.outputs:context", "PublishPointCloud.inputs:context"),
                ("OnPlaybackTick.outputs:tick", "TFChassisToLidar.inputs:execIn"),
                ("Context.outputs:context", "TFChassisToLidar.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "TFChassisToLidar.inputs:timeStamp"),
            ],
        },
    )

    if domain_id:
        og.Controller.attribute(f"{GRAPH_PATH}/Context.inputs:domain_id").set(int(domain_id))
        og.Controller.attribute(f"{GRAPH_PATH}/Context.inputs:useDomainIDEnvVar").set(False)
