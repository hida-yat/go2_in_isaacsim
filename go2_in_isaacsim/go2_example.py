# Adapted from isaacsim.examples.interactive.quadruped.quadruped_example
# (the Spot Quadruped example), swapped to drive a unitree_rl_lab Go2 policy.

import carb
import numpy as np
import omni
import omni.appwindow  # Contains handle to keyboard
from isaacsim.examples.interactive.base_sample import BaseSample

from . import settings
from .go2 import Go2FlatTerrainPolicy


class Go2Example(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self._world_settings["stage_units_in_meters"] = 1.0
        self._world_settings["physics_dt"] = 1.0 / 200.0
        self._world_settings["rendering_dt"] = 8.0 / 200.0
        self._base_command = np.zeros(3)

        # bindings for keyboard to command, same layout as the Spot example.
        # Magnitudes are kept inside this checkpoint's trained command ranges
        # (see params/deploy.yaml: lin_vel_x=[-1,1], lin_vel_y=[-0.4,0.4], ang_vel_z=[-1,1]).
        self._input_keyboard_mapping = {
            "NUMPAD_8": [1.0, 0.0, 0.0],
            "UP": [1.0, 0.0, 0.0],
            "NUMPAD_2": [-1.0, 0.0, 0.0],
            "DOWN": [-1.0, 0.0, 0.0],
            "NUMPAD_6": [0.0, -0.4, 0.0],
            "RIGHT": [0.0, -0.4, 0.0],
            "NUMPAD_4": [0.0, 0.4, 0.0],
            "LEFT": [0.0, 0.4, 0.0],
            "NUMPAD_7": [0.0, 0.0, 1.0],
            "N": [0.0, 0.0, 1.0],
            "NUMPAD_9": [0.0, 0.0, -1.0],
            "M": [0.0, 0.0, -1.0],
        }

    def setup_scene(self) -> None:
        environment_usd_path = settings.get("environment_usd_path")
        if environment_usd_path:
            from isaacsim.core.utils.stage import add_reference_to_stage

            add_reference_to_stage(environment_usd_path, "/World/Environment")
        else:
            self._world.scene.add_default_ground_plane(
                z_position=0,
                name="default_ground_plane",
                prim_path="/World/defaultGroundPlane",
                static_friction=0.2,
                dynamic_friction=0.2,
                restitution=0.01,
            )
        self.go2 = Go2FlatTerrainPolicy(
            prim_path="/World/Go2",
            name="Go2",
            position=np.array([0, 0, 0.42]),
        )
        timeline = omni.timeline.get_timeline_interface()
        self._event_timer_callback = timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), self._timeline_timer_callback_fn
        )

    async def setup_post_load(self) -> None:
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._sub_keyboard_event)
        self._physics_ready = False
        if not self.get_world().physics_callback_exists("physics_step"):
            self.get_world().add_physics_callback("physics_step", callback_fn=self.on_physics_step)
        await self.get_world().play_async()

    async def setup_post_reset(self) -> None:
        self._physics_ready = False
        await self._world.play_async()
        if not self.get_world().physics_callback_exists("physics_step"):
            self.get_world().add_physics_callback("physics_step", callback_fn=self.on_physics_step)

    def on_physics_step(self, step_size) -> None:
        if self._physics_ready:
            self.go2.forward(step_size, self._base_command)
        else:
            self._physics_ready = True
            self.go2.initialize()
            self.go2.post_reset()
            self.go2.robot.set_joints_default_state(self.go2.default_pos)

    def _sub_keyboard_event(self, event, *args, **kwargs) -> bool:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command += np.array(self._input_keyboard_mapping[event.input.name])
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command -= np.array(self._input_keyboard_mapping[event.input.name])
        return True

    def _timeline_timer_callback_fn(self, event) -> None:
        if self.go2:
            self._physics_ready = False
        if not self.get_world().physics_callback_exists("physics_step"):
            self.get_world().add_physics_callback("physics_step", callback_fn=self.on_physics_step)

    def world_cleanup(self):
        self._event_timer_callback = None
        if self._world.physics_callback_exists("physics_step"):
            self._world.remove_physics_callback("physics_step")
