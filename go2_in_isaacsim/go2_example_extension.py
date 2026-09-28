# Adapted from isaacsim.examples.interactive.quadruped.quadruped_example_extension
# Registers the Go2 example in the same "Isaac Examples" browser window as Spot.

import asyncio
import os

import carb.settings
import omni.ext
import omni.kit.app
import omni.ui as ui
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.examples.interactive.base_sample import BaseSampleUITemplate
from omni.kit.window.preferences import register_page, select_page, show_preferences_window, unregister_page

from .go2_example import Go2Example
from .piper_example import PiperExample
from .preferences import Go2PolicyPreferences

# Launch-time only (not persistent): pass
# --/exts/go2_in_isaacsim/open_preferences_on_startup=true on the Isaac Sim
# command line (e.g. from the isaac_go2_run alias) to open Edit > Preferences
# on this extension's page at startup. The Robotics Examples browser has its
# own equivalent, --/exts/isaacsim.examples.browser/visible_after_startup=true.
SETTING_OPEN_PREFERENCES_ON_STARTUP = "/exts/go2_in_isaacsim/open_preferences_on_startup"


class Go2ExampleUITemplate(BaseSampleUITemplate):
    """BaseSampleUITemplate's stock panel only has Load/Reset -- Load
    disables itself after the first click and only re-enables when the
    stage closes (see base_sample_extension.py's on_stage_event), so
    picking up a Preferences change (a different Robot/Environment USD,
    ROS2 settings, ...) otherwise means File > New Stage by hand, or
    quitting and relaunching Isaac Sim entirely. Adds a "Clear World" button
    that does that stage-close for you, in one click."""

    def build_extra_frames(self):
        with self.get_extra_frames_handle():
            with ui.CollapsableFrame(title="Utilities", width=ui.Fraction(1), height=0, collapsed=False):
                with ui.VStack(spacing=5, height=0):
                    ui.Button(
                        "Clear World",
                        clicked_fn=self._on_clear_world,
                        tooltip="Closes the stage so Load can be pressed again -- use this after changing"
                        " Preferences (Robot/Environment USD, ROS2 settings, ...) to pick them up without"
                        " restarting Isaac Sim.",
                    )

    def _on_clear_world(self):
        asyncio.ensure_future(self._sample.clear_async())


class PiperExampleUITemplate(Go2ExampleUITemplate):
    """Go2ExampleUITemplate's Clear World, plus a Randomize Lemons button
    that re-scatters the lemons without resetting the arm."""

    def build_extra_frames(self):
        super().build_extra_frames()
        with self.get_extra_frames_handle():
            with ui.CollapsableFrame(title="Grasp Practice", width=ui.Fraction(1), height=0, collapsed=False):
                with ui.VStack(spacing=5, height=0):
                    ui.Button(
                        "Randomize Lemons",
                        clicked_fn=self._sample.randomize_lemons,
                        tooltip="Scatter the lemons to new random poses in front of the arm, leaving the arm as-is."
                        " (Reset also does this, and returns the arm to its initial pose.)",
                    )


class Go2ExampleExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self.example_name = "Go2"
        self.category = "Policy"

        self._preferences_page = register_page(Go2PolicyPreferences())
        if carb.settings.get_settings().get_as_bool(SETTING_OPEN_PREFERENCES_ON_STARTUP):
            asyncio.ensure_future(self._open_preferences_async())

        overview = "This example runs a Unitree Go2 flat terrain velocity policy"
        overview += " trained with unitree_rl_lab (Isaac Lab), deployed directly in Isaac Sim."
        overview += "\n\nRobot USD, environment USD, and policy checkpoint are configurable from"
        overview += " Edit > Preferences > Go2 Policy Example (e.g. point Robot USD at a Go2 USD"
        overview += " with sensors + an Action Graph added, or Environment USD at a custom world)."
        overview += "\n\tKeybord Input:"
        overview += "\n\t\tup arrow / numpad 8: Move Forward"
        overview += "\n\t\tdown arrow/ numpad 2: Move Reverse"
        overview += "\n\t\tleft arrow/ numpad 4: Move Left"
        overview += "\n\t\tright arrow / numpad 6: Move Right"
        overview += "\n\t\tN / numpad 7: Spin Counterclockwise"
        overview += "\n\t\tM / numpad 9: Spin Clockwise"
        overview += "\n\nOptional ROS2 bridge (Edit > Preferences > Go2 Policy Example > ROS2 Bridge):"
        overview += " subscribes cmd_vel to drive the robot (keyboard still overrides while a key is held),"
        overview += " and publishes odom, tf, joint_states, and /clock -- the topics nav2 expects from a mobile base."
        overview += "\n\nOptional Mid-360 lidar: pick 'Go2 with Mid-360' from the Preset dropdown under"
        overview += " Edit > Preferences > Go2 Policy Example > Assets > Robot USD, instead of the bare Go2."
        overview += " If ROS2 Bridge is also enabled, it's published as a PointCloud2 with a TF at its"
        overview += " actual mount transform."
        overview += "\n\nOptional Piper arm: pick 'Go2 with Mid-360 + Piper' from that same Preset dropdown."
        overview += " If ROS2 Bridge is also enabled, its joint_states are published and it's driven from"
        overview += " joint_command -- e.g. from a MoveIt FollowJointTrajectory-to-topic bridge."
        overview += "\n\nAfter changing any Preferences, use the 'Clear World' button below (or File > New"
        overview += " Stage) before Load to pick them up -- Isaac Sim's Load button only works once per stage."

        overview += "\n\nPress the 'Open in IDE' button to view the source code."

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "Quadruped: Unitree Go2",
            "doc_link": "",
            "overview": overview,
            "sample": Go2Example(),
        }

        ui_handle = Go2ExampleUITemplate(**ui_kwargs)

        get_browser_instance().register_example(
            name=self.example_name,
            execute_entrypoint=ui_handle.build_window,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

        self._register_piper_example(ext_id)
        return

    async def _open_preferences_async(self):
        # The Preferences window itself is only created once its extension
        # finishes starting up, which can be after this one -- wait a few
        # frames before showing it and selecting this extension's page.
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()
        show_preferences_window()
        await omni.kit.app.get_app().next_update_async()
        if self._preferences_page:
            select_page(self._preferences_page)

    def _register_piper_example(self, ext_id: str):
        self.piper_example_name = "Piper Grasp Practice"
        self.piper_category = "Manipulation"

        overview = "Arm-only mode: the Piper arm and its wrist D435 RealSense, fixed to the world at the"
        overview += " origin -- no Go2, no locomotion policy. Lemons (physics rigid bodies) are scattered"
        overview += " at random in front of the arm as grasp targets; Reset (or 'Randomize Lemons' below)"
        overview += " scatters a new layout."
        overview += "\n\nWith Edit > Preferences > Go2 Policy Example > ROS2 Bridge enabled, publishes/subscribes"
        overview += " the same Piper joint_states/joint_command (and hardware-compatible) topics and the same D435"
        overview += " RGB/Depth/PointCloud2/CameraInfo topics as 'Go2 with Mid-360 + Piper', plus /clock."
        overview += " The TF tree is rooted at world -> arm_base (no chassis)."
        overview += "\n\nThe floor is always the plain ground plane (Environment USD applies only to the Go2"
        overview += " example). Lemon count is set in Preferences; use 'Clear World' before Load to pick up changes."

        ui_handle = PiperExampleUITemplate(
            ext_id=ext_id,
            file_path=os.path.abspath(__file__),
            title="Manipulation: Piper Grasp Practice",
            doc_link="",
            overview=overview,
            sample=PiperExample(),
        )
        get_browser_instance().register_example(
            name=self.piper_example_name,
            execute_entrypoint=ui_handle.build_window,
            ui_hook=ui_handle.build_ui,
            category=self.piper_category,
        )

    def on_shutdown(self):
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)
        get_browser_instance().deregister_example(name=self.piper_example_name, category=self.piper_category)

        if self._preferences_page:
            unregister_page(self._preferences_page)
            self._preferences_page = None

        return
