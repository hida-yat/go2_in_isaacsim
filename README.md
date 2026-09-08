# go2_in_isaacsim

A pure-Isaac-Sim Isaac Sim extension that runs a [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab)
(Isaac Lab)-trained Unitree Go2 locomotion policy, in the same style as the
built-in **Isaac Examples > Policy > Spot** example — click Load, click Play,
drive with the keyboard. No standalone Python script to run.

See [`docs/README.md`](docs/README.md) for what's inside and how to configure it
(robot USD, environment USD, policy checkpoint — all editable from
`Edit > Preferences > Go2 Policy Example`, no code changes needed), including an
optional ROS2 bridge (cmd_vel in, odom/tf/joint_states/clock out) for driving this
from `nav2` or any other ROS2 stack, an optional `go2_with_mid360.usd` Robot
USD variant (published as PointCloud2 when ROS2 Bridge is on) for nav2's costmaps,
and an optional `go2_with_mid360_and_piper.usd` variant that also mounts an
[AgileX Piper](https://github.com/agilexrobotics/piper_isaac_sim) arm (joint_states
out / joint_command in, for driving from MoveIt or any other ROS2 stack).

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

- `data/Robots/Go2/usd/go2.usd` — bare Go2 USD, sourced from
  [unitree_model](https://github.com/unitreerobotics/unitree_model)'s
  `Go2/usd/` (converted from Unitree's URDF). Not modified.
- `data/Robots/Go2/usd/go2_with_mid360.usd` — the same robot with a Mid-360
  lidar mounted on its head (references `go2.usd` and
  `data/Sensors/Mid360/Mid360.usd` below), built by
  `tools/build_go2_with_mid360.py`; see
  [Mid-360 lidar](docs/README.md#mid-360-lidar-optional) in `docs/README.md`.
- `data/Sensors/Mid360/Mid360.usd` — a Livox Mid-360 CAD model, converted from
  a STEP file (`Isaac Sim`'s built-in CAD Converter) to USD for this repo.
- `data/Robots/Go2/usd/go2_with_mid360_and_piper.usd` — `go2_with_mid360.usd`
  plus a Piper arm mounted on the back (references `go2_with_mid360.usd` and
  `data/Robots/Piper/usd/piper.usd` below), built by
  `tools/build_go2_with_mid360_and_piper.py`; see
  [Piper arm](docs/README.md#piper-arm-optional) in `docs/README.md`.
- `data/Robots/Piper/usd/piper.usd` (+ `configuration/*.usd`) — a Piper arm
  with wrist camera, taken as-is from
  [piper_isaac_sim](https://github.com/agilexrobotics/piper_isaac_sim)'s
  `piper_description/urdf/piper_description_v100_realsense_camera_v2/`
  (URDF-imported to USD upstream). Not modified.
- `data/Policies/Go2/` — a `unitree_rl_lab`-trained flat-terrain velocity
  policy (`policy.pt` + the matching Isaac Lab `params/env.yaml`).

The Go2, Mid-360, and Piper models are third-party artifacts, not authored by
this repo. No license file is included here yet; check the terms of the
upstream projects ([unitree_model](https://github.com/unitreerobotics/unitree_ros),
[unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab),
[piper_isaac_sim](https://github.com/agilexrobotics/piper_isaac_sim), and
whatever your Mid-360 CAD source's own terms are) before redistributing
further, and add a LICENSE here once you've decided on one for this repo's
own code.

## Swapping in your own assets

Nothing under `data/` is hardcoded into the Python — it's just the shipped
default. To point at a Go2 USD with sensors + an Action Graph added, a
different training environment, or a different checkpoint, use
`Edit > Preferences > Go2 Policy Example` (persists across restarts, no repo
changes required). See `docs/README.md` for the full field list.
