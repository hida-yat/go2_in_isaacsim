# Settings tab for this extension, registered under Edit > Preferences.
# Adapted from isaacsim.asset.gen.conveyor.ui's ConveyorBuilderPreferences
# pattern (StringField + folder-picker icon + Reset, backed by carb's
# persistent settings) so the robot USD, environment USD, and policy
# checkpoint can be pointed at custom assets without touching any code.

import asyncio
from typing import Callable

import omni.ui as ui
from omni.kit.window.preferences import PreferenceBuilder

from . import settings


def _create_filepicker(title: str, click_apply_fn: Callable = None):
    from omni.kit.window.filepicker import FilePickerDialog

    async def on_click_handler(filename: str, dirname: str, dialog: FilePickerDialog):
        dirname = dirname.strip()
        if filename and dirname and not dirname.endswith("/"):
            dirname += "/"
        fullpath = f"{dirname}{filename}"
        if click_apply_fn:
            click_apply_fn(fullpath)
        dialog.hide()

    dialog = FilePickerDialog(
        title,
        allow_multi_selection=False,
        apply_button_label="Select",
        click_apply_handler=lambda filename, dirname: asyncio.ensure_future(
            on_click_handler(filename, dirname, dialog)
        ),
        click_cancel_handler=lambda filename, dirname: dialog.hide(),
    )
    return dialog


class Go2PolicyPreferences(PreferenceBuilder):
    def __init__(self):
        super().__init__("Go2 Policy Example")

    def build(self):
        with ui.VStack(height=0):
            with self.add_frame("Assets"):
                with ui.VStack(height=0, spacing=5):
                    for key, label, tooltip, _filter in settings.FIELDS:
                        widget = self._build_path_row(key, label, tooltip)
                        if key == "environment_usd_path":
                            self._build_environment_preset_row(widget)
            with self.add_frame("ROS2 Bridge"):
                with ui.VStack(height=0, spacing=5):
                    self._build_bool_row(*settings.ROS2_ENABLE_FIELD)
                    for key, label, tooltip in settings.ROS2_TOGGLE_FIELDS:
                        self._build_bool_row(key, label, tooltip)
                    for key, label, tooltip in settings.ROS2_TEXT_FIELDS:
                        self._build_text_row(key, label, tooltip)
            with self.add_frame("Mid-360 Lidar"):
                with ui.VStack(height=0, spacing=5):
                    self._build_bool_row(*settings.MID360_ENABLE_FIELD)
                    for key, label, tooltip in settings.MID360_TEXT_FIELDS:
                        self._build_text_row(key, label, tooltip)
            ui.Spacer(height=ui.Fraction(1))

    def _build_path_row(self, key: str, label: str, tooltip: str) -> ui.StringField:
        with ui.HStack(height=24, spacing=4):
            ui.Label(label, width=140, tooltip=tooltip)
            widget = ui.StringField(height=20, tooltip=tooltip)
            widget.model.set_value(settings.get(key))
            widget.model.add_end_edit_fn(lambda m, k=key: settings.set(k, m.get_value_as_string()))

            def browse(w=widget, k=key):
                def on_pick(path, w=w, k=k):
                    settings.set(k, path)
                    w.model.set_value(path)

                _create_filepicker(title=f"Select {label}", click_apply_fn=on_pick)

            ui.Button(" ... ", clicked_fn=browse, width=24, tooltip="Browse")

            def reset(w=widget, k=key):
                settings.reset(k)
                w.model.set_value(settings.get(k))

            ui.Button("Reset", clicked_fn=reset, width=50)
        return widget

    def _build_environment_preset_row(self, path_widget: ui.StringField) -> None:
        """Quick-pick dropdown for settings.ENVIRONMENT_PRESETS, sitting right
        under the Environment USD path field. Picking a preset writes its
        (Nucleus-relative) path into that field; "Custom..." leaves it alone."""
        with ui.HStack(height=24, spacing=4):
            ui.Label("Preset", width=140, tooltip="Quick-pick one of Isaac Sim's bundled environments.")
            labels = [preset_label for preset_label, _ in settings.ENVIRONMENT_PRESETS]
            combo_model = ui.ComboBox(0, *labels, height=20).model

            def on_changed(model, _item, w=path_widget):
                idx = model.get_item_value_model().as_int
                value = settings.ENVIRONMENT_PRESETS[idx][1]
                if value is None:  # "Custom..." -- keep whatever is in the path field
                    return
                settings.set("environment_usd_path", value)
                w.model.set_value(value)

            combo_model.add_item_changed_fn(on_changed)

    def _build_text_row(self, key: str, label: str, tooltip: str) -> None:
        with ui.HStack(height=24, spacing=4):
            ui.Label(label, width=140, tooltip=tooltip)
            widget = ui.StringField(height=20, tooltip=tooltip)
            widget.model.set_value(settings.get(key))
            widget.model.add_end_edit_fn(lambda m, k=key: settings.set(k, m.get_value_as_string()))

            def reset(w=widget, k=key):
                settings.reset(k)
                w.model.set_value(settings.get(k))

            ui.Button("Reset", clicked_fn=reset, width=50)

    def _build_bool_row(self, key: str, label: str, tooltip: str) -> None:
        with ui.HStack(height=24, spacing=4):
            ui.Label(label, width=140, tooltip=tooltip)
            model = ui.SimpleBoolModel(default_value=settings.get(key) == "True")
            model.add_value_changed_fn(lambda m, k=key: settings.set(k, "True" if m.get_value_as_bool() else "False"))
            ui.CheckBox(model, width=20)
            ui.Spacer()
