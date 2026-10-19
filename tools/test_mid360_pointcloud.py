#!/usr/bin/env python3
"""Run with isaac_run tools/test_mid360_pointcloud.py (RTX + ROS2 required)."""
import os
import sys
import time
from pathlib import Path

os.environ["ROS_DOMAIN_ID"] = "84"
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
try:
    import numpy as np
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.ros2.bridge")
    enable_extension("isaacsim.sensors.rtx")
    enable_extension("go2_in_isaacsim")
    app.update()
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from go2_in_isaacsim import mid360, settings
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import FixedCuboid
    from isaacsim.core.nodes.bindings import _isaacsim_core_nodes
    from pxr import UsdGeom
    import rclpy
    from sensor_msgs.msg import PointCloud2

    # No edits to the user's persistent Preferences; isolated topic/domain.
    defaults = dict(settings.DEFAULTS)
    defaults.update(ros2_domain_id="84", ros2_namespace="lidar_test")
    settings.get = defaults.__getitem__
    world = World(physics_dt=0.005, rendering_dt=0.04, stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    # Closed box around the sensor so every ray returns.
    for i, (p, s) in enumerate([
        ((5, 0, 1), (0.2, 10, 4)), ((-5, 0, 1), (0.2, 10, 4)),
        ((0, 5, 1), (10, 0.2, 4)), ((0, -5, 1), (10, 0.2, 4)), ((0, 0, 3), (10, 10, 0.2)),
    ]):
        world.scene.add(FixedCuboid(prim_path=f"/World/Wall{i}", name=f"wall{i}",
                                    position=np.array(p, float), scale=np.array(s, float)))
    stage = world.stage
    UsdGeom.Xform.Define(stage, "/World/Body")
    mount = UsdGeom.Xform.Define(stage, "/World/Body/Mid360")
    mount.AddTranslateOp().Set((0.0, 0.0, 0.5))
    sensor = mid360.create_sensor("Mid360Sensor", "/World/Body/Mid360")
    publisher = mid360.publish_to_ros2(sensor, "/World/Body", "base")
    rclpy.init()
    node = rclpy.create_node("mid360_pointcloud_test")
    msgs = []
    node.create_subscription(PointCloud2, "/lidar_test/" + settings.get("mid360_topic"), msgs.append, 10)
    core_nodes = _isaacsim_core_nodes.acquire_interface()

    def run(seconds):
        for _ in range(int(seconds / 0.04)):
            world.step(render=True)
            rclpy.spin_once(node, timeout_sec=0.0)
            # Give the renderer slack, like the GUI's rate-limited main loop:
            # an unthrottled headless loop outruns it, and the RTX lidar then
            # delivers some frames' chunks twice (losing the chunk between).
            time.sleep(0.03)

    world.reset()
    run(3.0)
    for _ in range(20):
        time.sleep(0.01)
        rclpy.spin_once(node, timeout_sec=0.0)
    sim_now = core_nodes.get_sim_time_monotonic()

    assert len(msgs) >= 20, f"expected ~10 Hz scans over 3 s, got {len(msgs)}"
    dtype = np.dtype(mid360._POINT_DTYPE)
    for msg in msgs:
        names = [f.name for f in msg.fields]
        assert names == ["x", "y", "z", "intensity", "time", "ring"], names
        assert msg.header.frame_id == settings.get("mid360_frame_id")
        assert msg.point_step == dtype.itemsize and msg.row_step == msg.point_step * msg.width
    scans = [np.frombuffer(bytes(m.data), dtype=dtype) for m in msgs]
    assert all(len(p) == m.width for p, m in zip(scans, msgs))

    # Whole rotations: 360deg of azimuth, ~20,000 returns (40 ch x 500 steps).
    # Allow for an occasional chunk lost upstream (see run()).
    def coverage(points):
        return np.unique(np.floor(np.degrees(np.arctan2(points["y"], points["x"]))).astype(int)).size
    whole = [p for p in scans if coverage(p) >= 355 and 19000 < len(p) < 20500]
    assert len(whole) >= 0.9 * len(scans), [(coverage(p), len(p)) for p in scans]
    assert all(len(p) < 20500 for p in scans), "a scan holds more than one rotation"

    points = whole[-1]
    assert set(np.unique(points["ring"])) == set(range(40)), np.unique(points["ring"])
    assert np.all(points["intensity"] >= 0) and np.all(points["intensity"] <= 255)
    assert np.ptp(points["intensity"]) > 1, "intensity looks unset"
    # Ring follows elevation: bottom ring hits the floor, top ring the ceiling.
    assert np.median(points["z"][points["ring"] == 0]) < -0.4
    assert np.median(points["z"][points["ring"] == 39]) > 0.0

    # Per-point time: whole render-frame steps (40 ms), from 0, within one
    # 100 ms rotation (which spans 3-4 frames).
    for p in scans:
        t = np.unique(np.round(p["time"], 4))
        assert t[0] == 0.0 and t[-1] <= 0.12 + 1e-3, t
        assert np.allclose(np.mod(t + 1e-3, 0.04), 1e-3, atol=2e-3), t
    stamps = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in msgs])
    assert np.all(np.diff(stamps) > 0), np.diff(stamps)
    rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
    assert abs(rate - 10.0) < 0.3, rate
    assert 0 < sim_now - stamps[-1] < 0.5, (sim_now, stamps[-1])
    print(f"PASS: {len(msgs)} scans ({len(whole)} whole), {len(points)} pts/scan, ring 0-39, "
          f"time steps {np.unique(np.round(points['time'], 3)).tolist()}, {rate:.2f} Hz", flush=True)

    publisher.shutdown()

    # The bundled Robot USD: find_sensor must locate the baked-in sensor and
    # its own CAD housing must not swallow the rays.
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
    publisher = mid360.publish_to_ros2(lidar_prim, go2.robot.prim_path, "base")
    world.reset()
    go2.initialize()
    go2.post_reset()
    go2.robot.set_joints_default_state(go2.default_pos)
    msgs.clear()
    run(2.0)
    assert len(msgs) >= 12, f"bundled Go2: only {len(msgs)} scans"
    points = np.frombuffer(bytes(msgs[-1].data), dtype=dtype)
    near = np.linalg.norm(np.stack([points["x"], points["y"], points["z"]], axis=1), axis=1) < 0.15
    assert len(points) > 1000 and near.mean() < 0.05, (len(points), near.mean())
    print(f"PASS: bundled Go2 {len(msgs)} scans, {len(points)} pts in the last", flush=True)

    publisher.shutdown()
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
