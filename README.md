# go2_in_isaacsim

A pure-Isaac-Sim Isaac Sim extension that runs a [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab)
(Isaac Lab)-trained Unitree Go2 locomotion policy, in the same style as the
built-in **Isaac Examples > Policy > Spot** example — click Load, click Play,
drive with the keyboard. No standalone Python script to run.

See [`docs/README.md`](docs/README.md) for what's inside and how to configure it
(robot USD, environment USD, policy checkpoint — all editable from
`Edit > Preferences > Go2 Policy Example`, no code changes needed), including an
optional ROS2 bridge (cmd_vel in, odom/tf/joint_states/clock out) for driving this
from `nav2` or any other ROS2 stack, and an optional Mid-360 lidar mount
(published as PointCloud2) for nav2's costmaps.

## Install

Clone (or copy) this directory into your Isaac Sim install's `exts/` folder:

```bash
git clone <this-repo-url> "$HOME/isaacsim/exts/go2_in_isaacsim"
```

(Replace `$HOME/isaacsim` with your actual Isaac Sim install path if different.
Alternatively, keep the clone anywhere and add its parent folder under
`Window > Extensions > (gear icon) > Extension Search Paths` instead of copying
into the install tree.)

Then in Isaac Sim:

1. `Window > Extensions`, search `go2_in_isaacsim`, enable it
   (toggle **Autoload** to have it enabled automatically next time).
2. `Isaac Examples > Policy > Go2` → **Load** → **Play**.
3. Drive with arrow keys / numpad (see the example's overview panel for the
   full key mapping).

If you also have a system ROS2 install (e.g. `source /opt/ros/humble/setup.bash`
in your shell rc file), see [`docs/README.md`](docs/README.md#ros2-bridge-optional-for-nav2)
for a launcher shell function that avoids a Python-version conflict this causes
with the ROS2 Bridge feature below.

## What's bundled

- `data/Robots/Go2/usd/` — Go2 USD, sourced from
  [unitree_model](https://github.com/unitreerobotics/unitree_model)'s
  `Go2/usd/` (converted from Unitree's URDF). Not modified.
- `data/Policies/Go2/` — a `unitree_rl_lab`-trained flat-terrain velocity
  policy (`policy.pt` + the matching Isaac Lab `params/env.yaml`).
- `data/lidar_configs/Livox/Mid360.json` — a custom RTX Lidar profile
  approximating the Livox Mid-360's spec (Isaac Sim ships no official one).
  Authored for this repo, not a Livox/Unitree asset.

Both are third-party artifacts, not authored by this repo. No license file is
included here yet; check the terms of the upstream projects
([unitree_model](https://github.com/unitreerobotics/unitree_ros),
[unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab)) before
redistributing further, and add a LICENSE here once you've decided on one for
this repo's own code.

## Swapping in your own assets

Nothing under `data/` is hardcoded into the Python — it's just the shipped
default. To point at a Go2 USD with sensors + an Action Graph added, a
different training environment, or a different checkpoint, use
`Edit > Preferences > Go2 Policy Example` (persists across restarts, no repo
changes required). See `docs/README.md` for the full field list.
