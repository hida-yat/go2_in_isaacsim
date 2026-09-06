# Persistent, user-editable configuration for this extension.
#
# Values live under carb's persistent settings tree, so once changed from the
# Settings window (Window > Go2 Policy Example > Settings) they are written to
# the user's own Kit user.config.json and survive restarts / extension
# updates. Nothing here needs to be edited in code to point this extension at
# a different robot USD (e.g. a go2.usd with sensors + an Action Graph added),
# a different environment USD, or a different trained checkpoint.

import os

import carb.settings

_ROOT = "/persistent/exts/go2_policy.example"

# Everything under data/ ships inside this extension (git repo) so it works
# out of the box on a fresh checkout, with no external paths and no
# configuration at all.
_EXT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_DEFAULT_POLICY_PATH = os.path.join(_EXT_ROOT, "data", "Policies", "Go2", "policy.pt")
_DEFAULT_POLICY_ENV_PATH = os.path.join(_EXT_ROOT, "data", "Policies", "Go2", "env.yaml")

# Bundled bare Go2 USD (no sensors). Point "robot_usd_path" at a
# sensor-equipped variant (e.g. a go2_with_sensors.usd containing
# cameras/lidar + an Action Graph publishing them) to swap it in without
# touching any code.
_DEFAULT_ROBOT_USD_PATH = os.path.join(_EXT_ROOT, "data", "Robots", "Go2", "usd", "go2.usd")

# Empty by default -> the example just spawns a flat ground plane.
# Set to a world/environment USD to load that instead.
_DEFAULT_ENVIRONMENT_USD_PATH = ""

DEFAULTS = {
    "robot_usd_path": _DEFAULT_ROBOT_USD_PATH,
    "environment_usd_path": _DEFAULT_ENVIRONMENT_USD_PATH,
    "policy_path": _DEFAULT_POLICY_PATH,
    "policy_env_path": _DEFAULT_POLICY_ENV_PATH,
}

# Order + display metadata for the settings window.
FIELDS = [
    ("robot_usd_path", "Robot USD", "The Go2 (or Go2 + sensors) USD file to spawn.", [("USD files", "*.usd*")]),
    (
        "environment_usd_path",
        "Environment USD",
        "Optional world/environment USD to load instead of the default ground plane. Leave empty for the default ground plane.",
        [("USD files", "*.usd*")],
    ),
    ("policy_path", "Policy (.pt)", "TorchScript policy checkpoint, e.g. unitree_rl_lab's exported/policy.pt.", [("TorchScript policy", "*.pt")]),
    (
        "policy_env_path",
        "Policy env.yaml",
        "The Isaac Lab params/env.yaml dumped alongside the checkpoint (defines joint gains/defaults).",
        [("YAML", "*.yaml")],
    ),
]


def get(key: str) -> str:
    value = carb.settings.get_settings().get(f"{_ROOT}/{key}")
    if value:
        return value
    return DEFAULTS[key]


def set(key: str, value: str) -> None:
    carb.settings.get_settings().set(f"{_ROOT}/{key}", value)


def reset(key: str) -> None:
    carb.settings.get_settings().set(f"{_ROOT}/{key}", DEFAULTS[key])
