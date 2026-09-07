# Adapted from isaacsim.examples.interactive.quadruped.quadruped_example_extension
# Registers the Go2 example in the same "Isaac Examples" browser window as Spot.

import os

import omni.ext
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.examples.interactive.base_sample import BaseSampleUITemplate
from omni.kit.window.preferences import register_page, unregister_page

from .go2_example import Go2Example
from .preferences import Go2PolicyPreferences


class Go2ExampleExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self.example_name = "Go2"
        self.category = "Policy"

        self._preferences_page = register_page(Go2PolicyPreferences())

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
        overview += "\n\nOptional Mid-360 lidar (Edit > Preferences > Go2 Policy Example > Mid-360 Lidar):"
        overview += " mounts an approximate Livox Mid-360 on the robot's head and, if ROS2 Bridge is also"
        overview += " enabled, publishes it as a PointCloud2 with a static TF from the chassis frame."

        overview += "\n\nPress the 'Open in IDE' button to view the source code."

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "Quadruped: Unitree Go2",
            "doc_link": "",
            "overview": overview,
            "sample": Go2Example(),
        }

        ui_handle = BaseSampleUITemplate(**ui_kwargs)

        get_browser_instance().register_example(
            name=self.example_name,
            execute_entrypoint=ui_handle.build_window,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

        return

    def on_shutdown(self):
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)

        if self._preferences_page:
            unregister_page(self._preferences_page)
            self._preferences_page = None

        return
