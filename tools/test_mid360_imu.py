#!/usr/bin/env python3
"""Run with isaac_run tools/test_mid360_imu.py (native physics + ROS2 required)."""
import os
import sys
import time
from pathlib import Path

os.environ["ROS_DOMAIN_ID"] = "83"
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
try:
    import numpy as np
    import omni.graph.core as og
    from pxr import Gf, UsdGeom
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.ros2.bridge")
    enable_extension("isaacsim.sensors.physics")
    enable_extension("go2_in_isaacsim")
    app.update()
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from go2_in_isaacsim import imu, mid360, settings
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicCuboid
    import rclpy
    from sensor_msgs.msg import Imu
    from tf2_msgs.msg import TFMessage

    # No edits to the user's persistent Preferences; isolated topic/domain.
    defaults = dict(settings.DEFAULTS)
    defaults.update(ros2_domain_id="83", ros2_namespace="imu_test")
    settings.get = defaults.__getitem__
    world = World(physics_dt=0.005, rendering_dt=0.04, stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    body = world.scene.add(DynamicCuboid(
        prim_path="/World/Body", name="body", position=np.array([0., 0., 0.5]),
        scale=np.array([1., 1., 1.]), mass=1.,
    ))
    stage = world.stage
    mount = UsdGeom.Xform.Define(stage, "/World/Body/Mid360")
    mount.AddTranslateOp().Set(Gf.Vec3d(0.17, 0, 0.12))
    mount.AddRotateYOp().Set(13.)
    lidar = UsdGeom.Xform.Define(stage, "/World/Body/Mid360/Mid360Sensor")
    lidar.AddOrientOp().Set(Gf.Quatf(0, 0, 1, 0))
    sensor = imu.publish_to_ros2(lidar.GetPrim(), "/World/Body", "base")
    # Rebuilding reuses the sensor, without accumulating duplicate prims/graphs.
    assert imu.publish_to_ros2(lidar.GetPrim(), "/World/Body", "base") == sensor
    t, q = mid360._relative_transform(sensor, "/World/Body")
    np.testing.assert_allclose(t, [0.17, 0, 0.12], atol=1e-6)
    np.testing.assert_allclose(q, [0, np.sin(np.deg2rad(6.5)), 0, np.cos(np.deg2rad(6.5))], atol=1e-6)
    rclpy.init()
    node = rclpy.create_node("go2_imu_integration_test")
    samples, transforms = [], []
    sub = node.create_subscription(Imu, "/imu_test/livox/imu", samples.append, 1000)
    tf_sub = node.create_subscription(TFMessage, "/imu_test/tf", transforms.append, 1000)

    def drain():
        for _ in range(32):
            rclpy.spin_once(node, timeout_sec=0.0)

    def run(steps):
        for i in range(steps):
            world.step(render=(i % 8 == 0))
            drain()

    world.reset()
    run(240)
    deadline = time.monotonic() + 5
    while not samples and time.monotonic() < deadline:
        run(40)
        time.sleep(0.01)
    assert samples, "No IMU messages received over ROS2"
    samples.clear()
    run(200)
    time.sleep(0.1)
    drain()
    assert len(samples) >= 180, f"Expected ~200 messages, got {len(samples)}"
    stamps = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in samples])
    assert np.all(np.diff(stamps) > 0), np.diff(stamps)
    np.testing.assert_allclose(np.median(np.diff(stamps)), 0.005, atol=1e-5)
    m = samples[-1]
    assert m.header.frame_id == "livox_imu_frame"
    assert m.orientation_covariance[0] == -1, m.orientation_covariance
    expected = np.array([-np.sin(np.deg2rad(13)), 0, np.cos(np.deg2rad(13))]) * 9.81
    actual = [m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]
    np.testing.assert_allclose(actual, expected, atol=0.15)
    np.testing.assert_allclose([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z], 0, atol=0.05)
    assert transforms, "No IMU TF received"
    tf = transforms[-1].transforms[0]
    assert tf.header.frame_id == "base" and tf.child_frame_id == "livox_imu_frame"
    np.testing.assert_allclose([tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w], q, atol=1e-6)

    # Freely rotating body: verify rad/s and the sensor-frame axis transform.
    body.set_world_pose(position=np.array([0., 0., 20.]), orientation=np.array([1., 0., 0., 0.]))
    body.set_linear_velocity(np.zeros(3))
    body.set_angular_velocity(np.array([0., 0., 1.]))
    samples.clear()
    run(40)
    time.sleep(0.05)
    drain()
    m = samples[-1]
    np.testing.assert_allclose([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z], expected / 9.81, atol=0.1)

    world.stop()
    time.sleep(0.05)
    drain()
    n = len(samples)
    for _ in range(5):
        app.update()
        drain()
    assert len(samples) == n, "IMU published while stopped"
    previous_stamp = samples[-1].header.stamp
    previous_stamp = previous_stamp.sec + previous_stamp.nanosec * 1e-9
    world.reset()
    world.play()
    run(80)
    drain()
    assert len(samples) > n, "IMU did not resume after reset"
    m = samples[-1]
    assert m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 > previous_stamp
    # Load the actual bundled articulation: the sensor must find its rigid-body
    # ancestor through the mount Xform, not only work on the cuboid fixture.
    world.stop()
    World.clear_instance()
    import omni.usd
    omni.usd.get_context().new_stage()
    world = World(physics_dt=0.005, rendering_dt=0.04, stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    from go2_in_isaacsim.go2 import Go2FlatTerrainPolicy
    robot_usd = str(Path(__file__).resolve().parents[1] / "data/Robots/Go2/usd/go2_with_mid360.usd")
    go2 = Go2FlatTerrainPolicy(prim_path="/World/Go2", usd_path=robot_usd, position=np.array([0., 0., 0.42]))
    lidar_prim = mid360.find_sensor(go2.robot.prim_path)
    assert lidar_prim is not None
    imu.publish_to_ros2(lidar_prim, go2.robot.prim_path, "base")
    world.reset()
    go2.initialize()
    go2.post_reset()
    go2.robot.set_joints_default_state(go2.default_pos)
    samples.clear()
    for i in range(120):
        go2.forward(0.005, np.zeros(3))
        world.step(render=(i % 8 == 0))
        drain()
    time.sleep(0.05)
    drain()
    assert len(samples) >= 100, f"Bundled Go2 IMU missing samples: {len(samples)}"
    values = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z,
                        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for m in samples])
    assert np.all(np.isfinite(values)), "Non-finite IMU readings on Go2"
    assert np.max(np.abs(values[:, :3])) > 1., "Go2 acceleration is unexpectedly empty"
    print("PASS: ROS2 IMU rate, gravity/gyro axes, TF, timestamps, rebuild, stop/reset and bundled Go2", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    world.stop()
except BaseException:
    import traceback
    traceback.print_exc()
    sys.stderr.flush()
    sys.stdout.flush()
    # Kit fast shutdown otherwise masks assertion failures with exit code 0.
    os._exit(1)
finally:
    app.close()
