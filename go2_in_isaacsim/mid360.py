# Creates a Mid-360-approximate RTX Lidar (create_sensor, used by
# tools/build_go2_with_mid360.py to bake one into a Robot USD) and publishes
# whichever one is on the loaded robot as a ROS2 PointCloud2, if any
# (find_sensor + publish_to_ros2, called from go2_example.py).
#
# There is no official Mid-360 profile bundled with Isaac Sim's RTX Lidar
# (isaacsim.sensors.rtx/data/lidar_configs/ has Velodyne/Ouster/Hesai/SICK/...
# but no Livox), so MID360_ATTRS below defines a custom scan pattern
# matching the real sensor's headline specs -- 360deg horizontal x -7..+52deg
# vertical FOV, ~40m range, ~200,000 points/sec, 905nm -- using 40
# evenly-spaced vertical channels swept through a full rotation ("ROTARY"
# scanType). This is an envelope match, not a reproduction of the real
# Mid-360's non-repetitive (rosette) scan pattern.
#
# IMPORTANT: the sensor is created as a native OmniLidar prim (via
# omni.kit.commands "IsaacSensorCreateRtxLidar" with no config, which falls
# through to rep.functional.create.omni_lidar internally) with our scan
# parameters authored directly as omni:sensor:Core:* attributes -- the same
# mechanism used to build Isaac Sim's own bundled Nucleus lidar assets
# (Velodyne/Ouster/...). An earlier version of this file instead created a
# plain Camera prim with IsaacRtxLidarSensorAPI applied and a
# "sensorModelConfig" string naming a JSON profile file
# (force_camera_prim=True, the isaacsim.sensors.rtx command's deprecated
# fallback for configs outside its hardcoded Nucleus-asset list) -- that path
# produced *zero* rays in this Isaac Sim version, verified by comparing
# against a real, working Nucleus lidar asset in a headless ray-cast test
# (a box at a known position was correctly hit by ~47% of returned points
# with this native approach; the Camera+sensorModelConfig approach returned
# no points at all, for either our own profile or a known-good vendor one).

import omni.graph.core as og
import omni.usd
from pxr import Usd, UsdGeom

from . import settings

_NUM_CHANNELS = 40
_ELEVATION_MIN_DEG = -7.0
_ELEVATION_MAX_DEG = 52.0
_SCAN_RATE_HZ = 10.0
_REPORT_RATE_HZ = 5000  # total point rate = report rate * channels = ~200,000 pts/sec

_LIDAR_APIS = ("IsaacRtxLidarSensorAPI", "OmniSensorGenericLidarCoreAPI")

GRAPH_PATH = "/World/Go2Mid360ROS2"


def _mid360_attrs() -> dict:
    """omni:sensor:Core:* attributes for a 40-channel rotary approximation of
    the Mid-360, in the exact form isaacsim.sensors.rtx's own bundled lidar
    assets (e.g. Velodyne_VLS128, Example_Rotary) use natively."""
    elevations = [
        round(_ELEVATION_MIN_DEG + (_ELEVATION_MAX_DEG - _ELEVATION_MIN_DEG) * i / (_NUM_CHANNELS - 1), 3)
        for i in range(_NUM_CHANNELS)
    ]
    report_period_ns = 1e9 / _REPORT_RATE_HZ
    fire_times = [round(report_period_ns * i / _NUM_CHANNELS) for i in range(_NUM_CHANNELS)]
    zeros = [0.0] * _NUM_CHANNELS
    return {
        "omni:sensor:Core:scanType": "ROTARY",
        "omni:sensor:Core:rayType": "IDEALIZED",
        "omni:sensor:Core:nearRangeM": 0.1,
        "omni:sensor:Core:farRangeM": 40.0,
        "omni:sensor:Core:rangeResolutionM": 0.002,
        "omni:sensor:Core:rangeAccuracyM": 0.02,
        "omni:sensor:Core:avgPowerW": 0.002,
        "omni:sensor:Core:minReflectance": 0.1,
        "omni:sensor:Core:waveLengthNm": 905.0,
        "omni:sensor:Core:pulseTimeNs": 4,
        "omni:sensor:Core:maxReturns": 1,
        "omni:sensor:Core:scanRateBaseHz": _SCAN_RATE_HZ,
        "omni:sensor:Core:reportRateBaseHz": _REPORT_RATE_HZ,
        "omni:sensor:Core:numberOfEmitters": _NUM_CHANNELS,
        "omni:sensor:Core:numberOfChannels": _NUM_CHANNELS,
        "omni:sensor:Core:intensityMappingType": "LINEAR",
        "omni:sensor:Core:rotationDirection": "CW",
        "omni:sensor:Core:intensityProcessing": "NORMALIZATION",
        "omni:sensor:Core:skipDroppingInvalidPoints": True,
        "omni:sensor:Core:startAzimuthOffsetDeg": 0.0,
        "omni:sensor:modelName": "Mid360",
        "omni:sensor:Core:emitterState:s001:azimuthDeg": [0.0] * _NUM_CHANNELS,
        "omni:sensor:Core:emitterState:s001:elevationDeg": elevations,
        "omni:sensor:Core:emitterState:s001:fireTimeNs": fire_times,
        "omni:sensor:Core:emitterState:s001:channelId": list(range(1, _NUM_CHANNELS + 1)),
        "omni:sensor:Core:emitterState:s001:distanceCorrectionM": zeros,
        "omni:sensor:Core:emitterState:s001:focalDistM": zeros,
        "omni:sensor:Core:emitterState:s001:focalSlope": zeros,
        "omni:sensor:Core:emitterState:s001:horOffsetM": zeros,
        "omni:sensor:Core:emitterState:s001:vertOffsetM": zeros,
    }


def is_available() -> bool:
    try:
        import omni.kit.app
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.sensors.rtx")
        return omni.kit.app.get_app().get_extension_manager().is_extension_enabled("isaacsim.sensors.rtx")
    except Exception:
        return False


def create_sensor(name: str, parent: str):
    """Creates the Mid-360 sensor prim (native OmniLidar, see module
    docstring) named `name` under `parent`, at that parent's local origin
    (identity transform -- no pose kwargs are passed, so no xformOps get
    authored at all, leaving the prim to simply inherit its parent's
    transform as-is). Local identity means azimuth 0deg/elevation 0deg
    points along local +X with +Z up -- the same convention Isaac Sim's own
    bundled lidar assets use, so no extra fixed correction is needed here.

    Uses omni.replicator.core's functional API directly rather than the
    omni.kit.commands "IsaacSensorCreateRtxLidar" wrapper: that wrapper
    joins `path` and `parent` into one nested path before falling through to
    this same underlying call, then passes the *whole joined path* as the
    prim's `name` -- name may not itself contain a "/", so this gets
    rejected and silently flattened into a mangled prim at the wrong
    location. Calling the functional API directly avoids that bug entirely.
    """
    import omni.replicator.core as rep

    prim = rep.functional.create.omni_lidar(name=name, parent=parent, **_mid360_attrs())
    return prim


def find_sensor(robot_prim_path: str):
    """Looks for an RTX Lidar sensor anywhere under the loaded robot --
    either convention (native OmniLidar, or the older Camera+
    IsaacRtxLidarSensorAPI style some Nucleus assets still use). Returns the
    first one found, or None if the loaded Robot USD doesn't have one (e.g.
    bare go2.usd)."""
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        return None
    for prim in Usd.PrimRange(robot_prim):
        if any(prim.HasAPI(api) for api in _LIDAR_APIS):
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
