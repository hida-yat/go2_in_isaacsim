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
# omni.replicator.core's rep.functional.create.omni_lidar) with our scan
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


# No orientation offset is applied to the sensor prim: the OmniLidar's local
# frame (+X forward, +Z up) is used as-is, and publish_to_ros2's TF is read
# from that same prim. An earlier 180deg-about-Y "fix" here was really
# compensating for _relative_transform() passing the TF quaternion in
# (w, x, y, z) order instead of (x, y, z, w); once that was fixed, the extra
# rotation left the sensor itself mounted upside down (rays cast -52..+7deg
# instead of -7..+52deg, and the cloud flipped in RViz).


def create_sensor(name: str, parent: str):
    """Creates the Mid-360 sensor prim (native OmniLidar, see module
    docstring) named `name` under `parent`, at that parent's local origin
    and orientation (the parent mount Xform carries the real mount tilt).

    Uses omni.replicator.core's functional API directly rather than the
    omni.kit.commands "IsaacSensorCreateRtxLidar" wrapper: that wrapper
    joins `path` and `parent` into one nested path before falling through to
    this same underlying call, then passes the *whole joined path* as the
    prim's `name` -- name may not itself contain a "/", so this gets
    rejected and silently flattened into a mangled prim at the wrong
    location. Calling the functional API directly avoids that bug entirely.
    """
    import omni.replicator.core as rep

    return rep.functional.create.omni_lidar(name=name, parent=parent, **_mid360_attrs())


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
    # ROS2PublishRawTransformTree expects (x, y, z, w), not Gf's (w, x, y, z).
    return translation, (imaginary[0], imaginary[1], imaginary[2], quat.GetReal())


# Per-point timing, as measured against this Isaac Sim version (5.1) in a
# headless probe -- the published cloud is built in Python from these facts
# rather than by isaacsim.ros2.bridge's own ROS2RtxLidarHelper, whose
# PointCloud2 carries x/y/z only (no intensity, ring or per-point time) and,
# without fullScan, one message per *render frame* (~144deg of rotation at
# this example's 25 Hz rendering), not one per 10 Hz rotation:
# - Each render frame's chunk of returns is ray-cast against a single scene
#   snapshot, sensor pose included, at that frame's simulation time (a sensor
#   translating at a known speed saw the same pose at both ends of a chunk).
#   So the honest per-point time is the chunk's frame time -- a step function
#   (0, 40, 80 ms within a scan) -- not a continuous sweep, which would make
#   a deskewing consumer (FAST-LIO etc.) "correct" motion that never happened.
# - The annotator's own per-point "timestamp" output is unusable: it comes
#   back with only half as many entries as there are points, and (like the
#   chunk-to-frame pairing in general, below) isn't reliably from the same
#   frame as the points it's delivered with. So it isn't requested at all.
# - Chunks don't reliably arrive one per frame: now and then one shows up a
#   frame early and is then delivered again, the chunk it displaced lost. The
#   points' own azimuths are what's trustworthy -- the scan pattern is fixed,
#   so a chunk's azimuth says exactly where in the sweep, and so *when* on
#   the sensor's clock, it was cast -- and _on_frame works from those.
# - Simulation time comes from each frame's render reference time (as
#   isaacsim.sensors.rtx's own LidarRtx does it), on the same monotonic clock
#   as IsaacReadSimulationTime with resetOnStop=False -- what every other
#   graph in this extension stamps with.
_ELEVATION_STEP_DEG = (_ELEVATION_MAX_DEG - _ELEVATION_MIN_DEG) / (_NUM_CHANNELS - 1)
# Scans are cut where the CW sweep crosses azimuth +-180deg (directly behind
# the sensor), like a Velodyne driver's default cut angle.
_SCAN_CUT_AZIMUTH_DEG = 180.0
# Anything spanning longer than this is a stitched-together mess (e.g. after a
# pause or dropped frames), not one rotation; drop it rather than publish it.
_MAX_SCAN_SPAN_S = 1.5 / _SCAN_RATE_HZ
_CLOCK_OFFSET_WINDOW = 25
# A chunk starting further than this *behind* the previous one's end is a
# repeat (a real one continues ~0.72deg, one firing step, ahead of it).
_STALE_CHUNK_DEG = 5.0

# x, y, z, intensity, time, ring -- the field names (not offsets) are what
# velodyne_pointcloud's PointXYZIRT, and FAST-LIO/Point-LIO's Velodyne
# preprocessing (lidar_type 2, timestamp_unit 0 = seconds), match on.
_POINT_DTYPE = [
    ("x", "<f4"),
    ("y", "<f4"),
    ("z", "<f4"),
    ("intensity", "<f4"),
    ("time", "<f4"),
    ("ring", "<u2"),
    ("_pad", "<u2"),
]


class PointCloudPublisher:
    """Publishes one sensor_msgs/PointCloud2 per full 10 Hz rotation with
    per-point intensity (0-255), ring (channel 0-39, bottom to top) and time
    (seconds after header.stamp, which is the scan's first point's time).

    Uses its own rclpy Context so an explicit "ros2_domain_id" Preferences
    override is honored, the same as this extension's OmniGraph pipelines."""

    def __init__(self, sensor_prim_path: str, topic: str, frame_id: str, namespace: str, domain_id: str) -> None:
        import array
        import collections

        import numpy as np
        import omni.replicator.core as rep
        import rclpy
        from isaacsim.core.nodes.bindings import _isaacsim_core_nodes
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import PointCloud2, PointField

        self._np = np
        self._array = array
        self._PointCloud2 = PointCloud2
        self._dtype = np.dtype(_POINT_DTYPE)
        self._fields = [
            PointField(name=name, offset=self._dtype.fields[name][1], datatype=datatype, count=1)
            for name, datatype in (
                ("x", PointField.FLOAT32),
                ("y", PointField.FLOAT32),
                ("z", PointField.FLOAT32),
                ("intensity", PointField.FLOAT32),
                ("time", PointField.FLOAT32),
                ("ring", PointField.UINT16),
            )
        ]
        self._frame_id = frame_id
        self._core_nodes = _isaacsim_core_nodes.acquire_interface()

        self._context = rclpy.Context()
        rclpy.init(
            context=self._context,
            domain_id=int(domain_id) if domain_id else None,
            signal_handler_options=SignalHandlerOptions.NO,
        )
        self._node = rclpy.create_node("mid360_point_cloud", namespace=namespace or None, context=self._context)
        self._pub = self._node.create_publisher(PointCloud2, topic, 10)

        self._render_product = rep.create.render_product(
            sensor_prim_path, [1, 1], name="Mid360PointCloud", render_vars=["GenericModelOutput", "RtxSensorMetadata"]
        )
        self._reset_scan()
        self._last_phase = None
        self._rotation = None
        self._clock_offsets = collections.deque(maxlen=_CLOCK_OFFSET_WINDOW)

        # A Writer, not annotator.get_data() polled from an app-update
        # callback: polling was measured to drop and repeat chunks whenever
        # rendering runs ahead of Python (it did, with ROS2 enabled), while a
        # Writer's write() runs in the render pipeline once per frame, like
        # isaacsim.ros2.bridge's own writers, and carries that frame's own
        # reference time.
        publisher = self

        class _ChunkWriter(rep.Writer):
            def __init__(self):
                self.annotators = [
                    rep.AnnotatorRegistry.get_annotator(
                        "IsaacCreateRTXLidarScanBuffer",
                        init_params={
                            "enablePerFrameOutput": True,
                            "outputIntensity": True,
                            "outputAzimuth": True,
                            "outputElevation": True,
                        },
                    )
                ]

            def write(self, data):
                publisher._on_frame(data)

        self._writer = _ChunkWriter()
        self._writer.attach([self._render_product])

    def _reset_scan(self, from_cut: bool = False) -> None:
        self._chunks = []
        self._scan_start_time = None
        # Only scans that began at the cut angle are whole rotations (not the
        # partial one right after Play, or one restarted by _MAX_SCAN_SPAN_S).
        self._scan_from_cut = from_cut

    def _on_frame(self, payload) -> None:
        np = self._np
        data = next((v for k, v in payload.items() if k.startswith("IsaacCreateRTXLidarScanBuffer")), None)
        reference_time = payload.get("reference_time")
        if not data or len(data.get("azimuth", ())) == 0 or not reference_time or not reference_time[1]:
            return
        frame_time = self._core_nodes.get_sim_time_monotonic_at_time(tuple(reference_time))

        # The chunk's own azimuths are its clock (see above): total degrees
        # swept (CW, i.e. decreasing azimuth) past the cut angle, unwrapped
        # within the chunk (returns arrive in firing order) and continued
        # from the previous chunk's last return.
        azimuth = np.asarray(data["azimuth"], dtype=np.float64)
        phase = np.unwrap(np.mod(_SCAN_CUT_AZIMUTH_DEG - azimuth, 360.0), period=360.0)
        if self._last_phase is not None:
            gap = np.mod(phase[0] - self._last_phase + 180.0, 360.0) - 180.0
            if gap < -_STALE_CHUNK_DEG:
                return  # a repeat of a chunk already used
            phase += self._last_phase + gap - phase[0]
        # Sim time of the frame this chunk was cast in: the sweep runs at a
        # fixed rate, so phase is a sensor clock with a constant offset to
        # simulation time -- a running median of that offset ignores the
        # frames that deliver a chunk early or late.
        sweep_time = phase[0] / (360.0 * _SCAN_RATE_HZ)
        offset = frame_time - sweep_time
        if self._clock_offsets and abs(offset - self._clock_offset()) > _MAX_SCAN_SPAN_S:
            # Sensor restarted (e.g. Stop/Play) or a long stall: start over.
            self._reset_scan()
            self._clock_offsets.clear()
            self._rotation = None
            phase -= phase[0] - np.mod(phase[0], 360.0)
            offset = frame_time - phase[0] / (360.0 * _SCAN_RATE_HZ)
        self._clock_offsets.append(offset)
        chunk_time = phase[0] / (360.0 * _SCAN_RATE_HZ) + self._clock_offset()
        self._last_phase = phase[-1]

        n = len(azimuth)
        chunk = np.zeros(n, dtype=self._dtype)
        xyz = np.asarray(data["data"]).reshape(n, 3)
        chunk["x"], chunk["y"], chunk["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        chunk["intensity"] = np.asarray(data["intensity"]) * 255.0
        ring = np.rint((np.asarray(data["elevation"]) - _ELEVATION_MIN_DEG) / _ELEVATION_STEP_DEG)
        chunk["ring"] = np.clip(ring, 0, _NUM_CHANNELS - 1)

        # Returns right at the cut jitter back and forth across it; the first
        # crossing starts the new rotation for good.
        rotation = np.maximum.accumulate(np.floor(phase / 360.0))
        if self._rotation is None:
            self._rotation = rotation[0]
        rotation = np.maximum(rotation, self._rotation)

        start = 0
        for end in list(np.flatnonzero(np.diff(rotation)) + 1) + [n]:
            if rotation[start] != self._rotation:
                self._publish_scan(contiguous=rotation[start] == self._rotation + 1)
                self._rotation = rotation[start]
            self._append(chunk[start:end], chunk_time)
            start = end

    def _clock_offset(self) -> float:
        """Lower median: always one actually observed offset, so chunk times
        stay on the render-frame grid (a true median of an even count could
        average an on-time and an early frame into a time between frames)."""
        ordered = sorted(self._clock_offsets)
        return ordered[(len(ordered) - 1) // 2]

    def _append(self, points, chunk_time: float) -> None:
        if self._scan_start_time is None:
            self._scan_start_time = chunk_time
        elif chunk_time - self._scan_start_time > _MAX_SCAN_SPAN_S:
            self._reset_scan()
            self._scan_start_time = chunk_time
        points["time"] = chunk_time - self._scan_start_time
        self._chunks.append(points)

    def _publish_scan(self, contiguous: bool) -> None:
        """Ends the current rotation, publishing it if it's a whole one, and
        starts the next one at the cut angle -- unless whole rotations were
        skipped (contiguous=False), in which case the next one is a fresh
        start too."""
        if not (self._chunks and self._scan_from_cut):
            self._reset_scan(from_cut=contiguous)
            return
        points = self._np.concatenate(self._chunks)
        msg = self._PointCloud2()
        stamp_ns = int(round(self._scan_start_time * 1e9))
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        msg.header.frame_id = self._frame_id
        msg.height = 1
        msg.width = len(points)
        msg.fields = self._fields
        msg.is_bigendian = False
        msg.point_step = self._dtype.itemsize
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = self._array.array("B", points.tobytes())
        self._pub.publish(msg)
        self._reset_scan(from_cut=contiguous)

    def shutdown(self) -> None:
        try:
            self._writer.detach()
        except Exception:
            pass
        try:
            self._render_product.destroy()
        except Exception:
            pass
        self._node.destroy_node()
        self._context.try_shutdown()


def publish_to_ros2(sensor_prim, robot_prim_path: str, chassis_frame: str) -> PointCloudPublisher:
    """Publishes sensor_prim (from find_sensor) as a ROS2 PointCloud2 with
    per-point time/ring/intensity (see PointCloudPublisher), plus its
    chassis->lidar TF via an OmniGraph (computed from the prim's actual
    transform -- see _relative_transform -- on the same tf topic as
    ros2_bridge.py's world->odom->chassis chain, so it shows up connected to
    the rest of the tree). The caller owns the returned publisher and must
    shutdown() it when the scene is torn down."""
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
                ("TFChassisToLidar", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ],
            keys.SET_VALUES: [
                ("ReadSimTime.inputs:resetOnStop", False),
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
                ("OnPlaybackTick.outputs:tick", "TFChassisToLidar.inputs:execIn"),
                ("Context.outputs:context", "TFChassisToLidar.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "TFChassisToLidar.inputs:timeStamp"),
            ],
        },
    )

    if domain_id:
        og.Controller.attribute(f"{GRAPH_PATH}/Context.inputs:domain_id").set(int(domain_id))
        og.Controller.attribute(f"{GRAPH_PATH}/Context.inputs:useDomainIDEnvVar").set(False)

    return PointCloudPublisher(
        sensor_prim_path, settings.get("mid360_topic"), lidar_frame, node_namespace, domain_id
    )
