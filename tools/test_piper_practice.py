#!/usr/bin/env python3
"""Run with isaac_run tools/test_piper_practice.py (native physics + ROS2 required).

Headless check of the arm-only "Piper Grasp Practice" example: loads it the
same way the example browser's Load button does, then checks the fixed-base
arm, the lemons (spawn region, settling, Randomize/Reset), and its ROS2
interfaces (joint_states/joint_command, hardware-compatible topics, /clock,
world-rooted TF, D435 point cloud)."""
import asyncio
import os
import sys
import time
from pathlib import Path

os.environ["ROS_DOMAIN_ID"] = "84"
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
try:
    import numpy as np
    import omni.timeline
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.ros2.bridge")
    enable_extension("go2_in_isaacsim")
    app.update()
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from go2_in_isaacsim import lemons, settings
    from go2_in_isaacsim.piper_example import PiperExample
    import rclpy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState, PointCloud2
    from tf2_msgs.msg import TFMessage

    # No edits to the user's persistent Preferences; isolated topic/domain.
    defaults = dict(settings.DEFAULTS)
    defaults.update(
        ros2_enabled="True",
        ros2_domain_id="84",
        ros2_namespace="piper_test",
        environment_usd_path="",
        lemon_count="4",
    )
    settings.get = defaults.__getitem__

    def run_async(coro):
        task = asyncio.ensure_future(coro)
        while not task.done():
            app.update()
        task.result()

    rclpy.init()
    node = rclpy.create_node("piper_practice_test")
    received = {"joint_states": [], "hw": [], "clock": [], "tf": [], "points": []}
    node.create_subscription(JointState, "/piper_test/piper/joint_states", received["joint_states"].append, 100)
    node.create_subscription(JointState, "/piper_test/joint_states_single", received["hw"].append, 100)
    node.create_subscription(Clock, "/clock", received["clock"].append, 100)
    node.create_subscription(TFMessage, "/piper_test/tf", received["tf"].append, 100)
    node.create_subscription(PointCloud2, "/piper_test/realsense/depth/color/points", received["points"].append, 10)
    command_pub = node.create_publisher(JointState, "/piper_test/piper/joint_command", 10)

    sample = PiperExample()
    run_async(sample.load_world_async())
    world = sample.get_world()

    def run(steps):
        for i in range(steps):
            world.step(render=(i % 8 == 0))
            for _ in range(8):
                rclpy.spin_once(node, timeout_sec=0.0)

    def lemon_positions():
        return np.array([prim.get_world_pose()[0] for prim in sample._lemons])

    def check_in_spawn_region(positions):
        radii = np.linalg.norm(positions[:, :2], axis=1)
        assert np.all((radii > 0.2) & (radii < 0.5)), radii
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                assert np.linalg.norm(positions[i, :2] - positions[j, :2]) > 0.08, (i, j, positions)

    # Load leaves the world paused, with lemons already scattered.
    assert len(sample._lemons) == 4
    before_play = lemon_positions()
    check_in_spawn_region(before_play)

    omni.timeline.get_timeline_interface().play()
    run(400)  # 2s of sim time

    arm = sample._arm
    dofs = list(arm.dof_names)
    assert dofs == [f"joint{i}" for i in range(1, 9)], dofs
    # Fixed base: arm_base stays at the origin.
    from pxr import UsdGeom

    arm_base = world.stage.GetPrimAtPath("/World/Piper/arm_base")
    base_pos = UsdGeom.Xformable(arm_base).ComputeLocalToWorldTransform(0).ExtractTranslation()
    np.testing.assert_allclose(base_pos, [0, 0, 0], atol=1e-3)

    # Lemons settled on the ground, lying on their side, near where they spawned.
    settled = lemon_positions()
    check_in_spawn_region(settled)
    np.testing.assert_allclose(settled[:, 2], lemons._SEMI_AXES[2], atol=0.006)
    assert np.all(np.linalg.norm(settled[:, :2] - before_play[:, :2], axis=1) < 0.03), settled - before_play
    velocities = np.array([prim.get_linear_velocity() for prim in sample._lemons])
    assert np.all(np.linalg.norm(velocities, axis=1) < 0.01), velocities

    # Randomize while playing: new layout, still valid, still settles.
    sample.randomize_lemons()
    run(200)
    randomized = lemon_positions()
    check_in_spawn_region(randomized)
    assert np.any(np.linalg.norm(randomized[:, :2] - settled[:, :2], axis=1) > 0.02)

    # ROS2: telemetry, clock, world-rooted TF, D435 point cloud.
    deadline = time.monotonic() + 10
    while not all(received.values()) and time.monotonic() < deadline:
        run(40)
    missing = [k for k, v in received.items() if not v]
    assert not missing, f"no messages received on: {missing}"
    assert list(received["hw"][-1].name) == [f"joint{i}" for i in range(1, 7)] + ["gripper"]
    tf_edges = {(t.header.frame_id, t.child_frame_id) for msg in received["tf"] for t in msg.transforms}
    assert ("world", "arm_base") in tf_edges, sorted(tf_edges)
    assert ("link6", settings.get("realsense_frame_id")) in tf_edges, sorted(tf_edges)
    assert received["points"][-1].width * received["points"][-1].height > 0

    # joint_command drives the arm.
    command = JointState()
    command.name = ["joint1"]
    command.position = [0.5]
    for _ in range(300):
        command_pub.publish(command)
        run(1)
    joint1 = arm.get_joint_positions()[0]
    assert abs(joint1 - 0.5) < 0.05, joint1

    # Reset: arm back to its initial pose, a fresh lemon layout.
    run_async(sample.reset_async())
    run(20)
    assert abs(arm.get_joint_positions()[0]) < 0.05, arm.get_joint_positions()
    after_reset = lemon_positions()
    check_in_spawn_region(after_reset)
    assert np.any(np.linalg.norm(after_reset[:, :2] - randomized[:, :2], axis=1) > 0.02)

    node.destroy_node()
    rclpy.shutdown()
    print("PIPER PRACTICE TEST PASSED")
    with open(os.environ.get("PIPER_TEST_RESULT", "/dev/null"), "w") as f:
        f.write("PASSED\n")
except Exception:
    import traceback

    with open(os.environ.get("PIPER_TEST_RESULT", "/dev/null"), "w") as f:
        f.write(traceback.format_exc())
    raise
finally:
    app.close()
