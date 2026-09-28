# Arm-only mode: the Piper arm (+ its wrist D435) on its own, fixed to the
# world at the origin -- no Go2, no locomotion policy, nothing that moves
# the base. A grasp-practice area: physics lemons scattered in front of the
# arm (see lemons.py), re-scattered on every Reset.
#
# Uses the bare data/Robots/Piper/usd/piper.usd, not a Go2 variant: its
# root_joint already ships welding arm_base to the world (body0 empty) --
# tools/build_go2_with_mid360_and_piper.py is what redirects that same joint
# onto Go2's base -- so referencing it as-is gives a fixed-base arm.
#
# ROS2 interfaces are exactly the Go2+Piper ones (piper.publish_to_ros2,
# piper.HardwareCompatibleBridge, realsense.publish_to_ros2, same topic
# Preferences), so a MoveIt/piper_ros setup built against one works against
# the other. Only the TF root differs: world -> arm_base (no chassis).

import os

import carb
import numpy as np
import omni
from isaacsim.examples.interactive.base_sample import BaseSample

from . import environment, lemons, piper, realsense, ros2_bridge, settings

PIPER_USD_PATH = os.path.join(settings.EXT_ROOT, "data", "Robots", "Piper", "usd", "piper.usd")
MOUNT_PATH = "/World/Piper"
CLOCK_GRAPH_PATH = "/World/PiperClockROS2"
_LOG_PREFIX = "Piper Grasp Practice"


class PiperExample(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        # Same physics/render rates as the Go2 example, so the D435 streams
        # and joint_states come out at the same rates in both modes.
        self._world_settings["stage_units_in_meters"] = 1.0
        self._world_settings["physics_dt"] = 1.0 / 200.0
        self._world_settings["rendering_dt"] = 8.0 / 200.0
        self._arm = None
        self._physics_ready = False
        self._event_timer_callback = None
        self._lemons = []
        self._rng = np.random.default_rng()
        self._ros2_enabled = False
        self._piper_hw_bridge = None

    def setup_scene(self) -> None:
        from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
        from isaacsim.core.utils.stage import add_reference_to_stage

        # Deliberately not environment.load(): the grasp-practice area is
        # just a flat floor + the lemons -- a larger Environment USD (e.g.
        # the Lemon Tree preset) only adds clutter to the D435's view/
        # OctoMap here. That Preference still applies to the Go2 example.
        environment.load_ground_plane(self._world)
        add_reference_to_stage(PIPER_USD_PATH, MOUNT_PATH)
        stage = omni.usd.get_context().get_stage()

        arm_prim = piper.find_arm_in_mount(MOUNT_PATH)
        if arm_prim is None:
            carb.log_error(f"{_LOG_PREFIX}: no articulation root found under {MOUNT_PATH} ({PIPER_USD_PATH}).")
            return
        self._arm = self._world.scene.add(SingleArticulation(prim_path=arm_prim.GetPath().pathString, name="Piper"))

        self._lemons = [
            self._world.scene.add(SingleRigidPrim(prim_path=path, name=path.rsplit("/", 1)[-1]))
            for path in lemons.spawn(stage, int(settings.get("lemon_count")))
        ]

        self._ros2_enabled = settings.get("ros2_enabled") == "True"
        if self._ros2_enabled and not ros2_bridge.is_available():
            carb.log_warn(
                f"{_LOG_PREFIX}: ROS2 Bridge is enabled in Preferences but isaacsim.ros2.bridge could not be "
                "enabled (no compatible ROS2 environment found?). Skipping all ROS2 publishing this run."
            )
            self._ros2_enabled = False
        if self._ros2_enabled:
            if settings.get("ros2_publish_clock") == "True":
                ros2_bridge.build_clock_graph(CLOCK_GRAPH_PATH)
            piper.publish_to_ros2(arm_prim, None)
            camera_mount_prim = realsense.find_sensor_in_mount(MOUNT_PATH)
            if camera_mount_prim is not None:
                realsense.publish_to_ros2(camera_mount_prim)

        timeline = omni.timeline.get_timeline_interface()
        self._event_timer_callback = timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), self._timeline_timer_callback_fn
        )

    async def setup_post_load(self) -> None:
        self._physics_ready = False
        self.randomize_lemons()
        if not self.get_world().physics_callback_exists("physics_step"):
            self.get_world().add_physics_callback("physics_step", callback_fn=self.on_physics_step)
        # Same as the Go2 example: Load leaves the world paused -- press Play.

    async def setup_post_reset(self) -> None:
        self._physics_ready = False
        # World.reset_async just restored every lemon to its default state
        # (the previous layout) -- scatter a fresh one for the next attempt.
        self.randomize_lemons()
        await self._world.play_async()
        if not self.get_world().physics_callback_exists("physics_step"):
            self.get_world().add_physics_callback("physics_step", callback_fn=self.on_physics_step)

    def randomize_lemons(self) -> None:
        if self._lemons:
            lemons.randomize(self._lemons, self._rng)

    def on_physics_step(self, step_size) -> None:
        if self._arm is None:
            return
        if self._physics_ready:
            if self._piper_hw_bridge is not None:
                self._piper_hw_bridge.step()
        else:
            # Same first-tick re-initialize as go2_example.py: after a
            # Stop/Play the articulation's physics handle is stale.
            self._physics_ready = True
            self._arm.initialize()
            if self._ros2_enabled and self._piper_hw_bridge is None:
                self._piper_hw_bridge = piper.build_hardware_compatible_bridge(self._arm)

    def _timeline_timer_callback_fn(self, event) -> None:
        self._physics_ready = False
        if not self.get_world().physics_callback_exists("physics_step"):
            self.get_world().add_physics_callback("physics_step", callback_fn=self.on_physics_step)

    def world_cleanup(self):
        self._event_timer_callback = None
        if self._piper_hw_bridge is not None:
            self._piper_hw_bridge.shutdown()
            self._piper_hw_bridge = None
        self._arm = None
        self._lemons = []
        if self._world.physics_callback_exists("physics_step"):
            self._world.remove_physics_callback("physics_step")
