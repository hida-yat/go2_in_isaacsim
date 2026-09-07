# Mounts an approximate Livox Mid-360 RTX Lidar on the robot's head and
# (optionally, alongside the ROS2 Bridge) publishes it as a ROS2 PointCloud2.
#
# There is no official Mid-360 profile bundled with Isaac Sim's RTX Lidar
# (isaacsim.sensors.rtx/data/lidar_configs/ has Velodyne/Ouster/Hesai/SICK/...
# but no Livox), so this ships a custom profile (data/lidar_configs/Livox/
# Mid360.json, "rotary" scanType) matching the real sensor's headline specs
# -- 360deg horizontal x -7..+52deg vertical FOV, ~40m range, ~200,000
# points/sec, 905nm -- using 40 evenly-spaced vertical channels swept through
# a full rotation. This is an envelope match, not a reproduction of the real
# Mid-360's non-repetitive (rosette) scan pattern.
#
# The lidar is created directly as an RTX "camera" prim (IsaacSensorSchema's
# IsaacRtxLidarSensorAPI applied to a UsdGeom.Camera), the same mechanism
# isaacsim.sensors.rtx's own IsaacSensorCreateRtxLidar command falls back to
# (force_camera_prim=True) for any sensor that isn't one of its bundled
# Nucleus-hosted vendor assets -- this is the only way to point the renderer
# at an arbitrary custom JSON scan profile in this Isaac Sim version.

import math
import os

import carb.settings
import omni.kit.commands
import omni.graph.core as og
import omni.usd
from isaacsim.core.utils.xforms import reset_and_set_xform_ops
from pxr import Gf

from . import settings

_CONFIG_NAME = "Mid360"
_CONFIG_DIR = os.path.join(settings.EXT_ROOT, "data", "lidar_configs", "Livox")
_PROFILE_SEARCH_PATH_SETTING = "/app/sensors/nv/lidar/profileBaseFolder"

SENSOR_NAME = "Mid360"
GRAPH_PATH = "/World/Go2Mid360ROS2"


def _parse_xyz(text: str, default=(0.0, 0.0, 0.0)):
    try:
        parts = [float(p) for p in text.split(",")]
        if len(parts) == 3:
            return tuple(parts)
    except (TypeError, ValueError):
        pass
    return default


def _ensure_profile_search_path() -> None:
    """Adds this extension's data/lidar_configs/Livox/ to the RTX lidar
    plugin's profile search path (additive -- preserves Isaac Sim's own
    vendor config directories already registered there)."""
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


def mount(robot_prim_path: str):
    """Creates (or replaces) the Mid-360 sensor prim as a child of the
    robot's articulation root. Returns the created sensor Usd.Prim."""
    _ensure_profile_search_path()

    stage = omni.usd.get_context().get_stage()
    sensor_path = f"{robot_prim_path}/{SENSOR_NAME}"
    if stage.GetPrimAtPath(sensor_path).IsValid():
        stage.RemovePrim(sensor_path)

    _, sensor_prim = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=sensor_path,
        parent=None,
        config=_CONFIG_NAME,
        force_camera_prim=True,
    )

    tx, ty, tz = _parse_xyz(settings.get("mid360_translate"), (0.28, 0.0, 0.10))
    tilt_deg = float(settings.get("mid360_tilt_deg") or 0.0)
    half = math.radians(tilt_deg) / 2.0
    # Pitch about the robot's Y (left) axis. Gf.Quatd is (w, i, j, k).
    orientation = Gf.Quatd(math.cos(half), 0.0, math.sin(half), 0.0)
    reset_and_set_xform_ops(sensor_prim, Gf.Vec3d(tx, ty, tz), orientation)

    return sensor_prim


def publish_to_ros2(sensor_prim_path: str, chassis_frame: str) -> None:
    """Builds the OmniGraph that publishes the mounted Mid-360 as a ROS2
    PointCloud2, plus a static chassis->lidar TF at its mount offset."""
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(GRAPH_PATH).IsValid():
        stage.RemovePrim(GRAPH_PATH)

    node_namespace = settings.get("ros2_namespace")
    domain_id = settings.get("ros2_domain_id")
    lidar_frame = settings.get("mid360_frame_id")
    tx, ty, tz = _parse_xyz(settings.get("mid360_translate"), (0.28, 0.0, 0.10))
    tilt_deg = float(settings.get("mid360_tilt_deg") or 0.0)
    half = math.radians(tilt_deg) / 2.0
    # og.Controller SET_VALUES wants a plain (w, i, j, k) tuple for quatd
    # attributes, not a Gf.Quatd object (that's only for direct USD xform ops).
    orientation = (math.cos(half), 0.0, math.sin(half), 0.0)

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
                ("TFChassisToLidar.inputs:translation", Gf.Vec3d(tx, ty, tz)),
                ("TFChassisToLidar.inputs:rotation", orientation),
                ("TFChassisToLidar.inputs:staticPublisher", True),
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
