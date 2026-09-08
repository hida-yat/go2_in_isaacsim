"""Ideal physics IMU at the Mid-360 mount, published at the physics rate.

The mount origin approximates the internal IMU position. Its axes follow the
mount, independently of the RTX lidar's rendering-axis correction. No hardware
bias/noise model or ground-truth attitude is added to the raw IMU message.
"""

import omni.graph.core as og
import omni.kit.commands
import omni.usd
from pxr import Gf, Sdf

from . import mid360, settings

GRAPH_PATH = "/World/Go2Mid360IMUROS2"


def publish_to_ros2(lidar_prim, robot_prim_path: str, chassis_frame: str):
    """Create a runtime IMU below the lidar's mount and its ROS2/TF graph.

    Existing robot USDs work without rebuilding. Call before starting physics.
    Sampling follows each physics step (200 Hz in Go2Example), not rendering.
    """
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.sensors.physics")
    stage = lidar_prim.GetStage()
    mount_path = lidar_prim.GetParent().GetPath().pathString
    imu_path = mount_path + "/Mid360Imu"
    imu_frame = settings.get("mid360_imu_frame_id")
    if imu_frame in (chassis_frame, settings.get("mid360_frame_id")):
        raise ValueError("IMU frame must differ from the chassis and lidar frames")

    if stage.GetPrimAtPath(GRAPH_PATH).IsValid():
        stage.RemovePrim(GRAPH_PATH)
    imu_prim = stage.GetPrimAtPath(imu_path)
    if not imu_prim.IsValid():
        success, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/Mid360Imu",
            parent=mount_path,
            sensor_period=0.0,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            linear_acceleration_filter_size=1,
            angular_velocity_filter_size=1,
            orientation_filter_size=1,
        )
        if not success or sensor is None:
            raise RuntimeError(f"Could not create physics IMU at {imu_path}")
        imu_prim = sensor.GetPrim()
    if imu_prim.GetTypeName() != "IsaacImuSensor":
        raise ValueError(f"{imu_path} exists but is not an IsaacImuSensor")

    translation, rotation = mid360._relative_transform(imu_prim, robot_prim_path)
    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": GRAPH_PATH,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("PhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
                ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
                ("TFChassisToIMU", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ],
            keys.SET_VALUES: [
                ("ReadIMU.inputs:imuPrim", [Sdf.Path(imu_path)]),
                ("ReadIMU.inputs:readGravity", True),
                ("ReadIMU.inputs:useLatestData", True),
                # Same monotonic simulation clock as the lidar/robot graphs.
                ("ReadSimTime.inputs:resetOnStop", False),
                ("PublishIMU.inputs:topicName", settings.get("mid360_imu_topic")),
                ("PublishIMU.inputs:frameId", imu_frame),
                ("PublishIMU.inputs:nodeNamespace", settings.get("ros2_namespace")),
                ("PublishIMU.inputs:publishOrientation", False),
                ("PublishIMU.inputs:publishAngularVelocity", True),
                ("PublishIMU.inputs:publishLinearAcceleration", True),
                ("TFChassisToIMU.inputs:parentFrameId", chassis_frame),
                ("TFChassisToIMU.inputs:childFrameId", imu_frame),
                ("TFChassisToIMU.inputs:translation", translation),
                ("TFChassisToIMU.inputs:rotation", rotation),
                ("TFChassisToIMU.inputs:topicName", settings.get("ros2_tf_topic")),
                ("TFChassisToIMU.inputs:nodeNamespace", settings.get("ros2_namespace")),
            ],
            keys.CONNECT: [
                ("PhysicsStep.outputs:step", "ReadIMU.inputs:execIn"),
                ("ReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),
                ("ReadIMU.outputs:execOut", "TFChassisToIMU.inputs:execIn"),
                ("ReadIMU.outputs:linAcc", "PublishIMU.inputs:linearAcceleration"),
                ("ReadIMU.outputs:angVel", "PublishIMU.inputs:angularVelocity"),
                ("ReadSimTime.outputs:simulationTime", "PublishIMU.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "TFChassisToIMU.inputs:timeStamp"),
                ("Context.outputs:context", "PublishIMU.inputs:context"),
                ("Context.outputs:context", "TFChassisToIMU.inputs:context"),
            ],
        },
    )
    domain_id = settings.get("ros2_domain_id")
    if domain_id:
        og.Controller.attribute(f"{GRAPH_PATH}/Context.inputs:domain_id").set(int(domain_id))
        og.Controller.attribute(f"{GRAPH_PATH}/Context.inputs:useDomainIDEnvVar").set(False)
    return imu_prim
