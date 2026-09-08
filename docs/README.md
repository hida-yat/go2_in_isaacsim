# Go2 Policy Example

Runs a `unitree_rl_lab`-trained Go2 flat-terrain velocity policy in pure Isaac Sim,
using the same `PolicyController` base class as the built-in Spot Quadruped example
(`isaacsim.robot.policy.examples`).

Open via **Isaac Examples > Policy > Go2**, click **Load**, then **Play**.
Drive with arrow keys / numpad (same mapping as Spot).

Isaac Sim's **Load** button only works once per stage -- clicking it again after
changing something (in Preferences, or just wanting a fresh run) does nothing.
Click **Clear World** (in this example's own panel, below Load/Reset) first --
it closes the stage, which is what actually re-enables Load -- then **Load**
again. (Equivalent to `File > New Stage`, just without leaving the panel.)

## Configuration

Nothing here is hardcoded in Python. Open **Edit > Preferences > Go2 Policy Example**
to point this extension at your own assets:

- **Robot USD** — the Go2 USD to spawn. Defaults to the bare `go2.usd`. The **Preset**
  dropdown right below it quick-fills this with either that bare file or the bundled
  `go2_with_mid360.usd` (see [Mid-360 lidar](#mid-360-lidar-optional) below); pick
  **Custom...** to type/browse your own variant instead (e.g. with other sensors +
  an Action Graph added).
- **Environment USD** — optional world/environment USD. Leave empty for the default
  flat ground plane. The **Preset** dropdown right below it quick-fills this field
  with one of Isaac Sim's own bundled sample environments (Grid, Simple Room,
  Warehouse, ...), resolved against your Isaac Sim assets root at Load time; pick
  **Custom...** to type/browse your own path instead.
- **Policy (.pt)** / **Policy env.yaml** — the TorchScript checkpoint and its matching
  Isaac Lab `params/env.yaml` (joint gains, default pose, action/observation scales).
  A default checkpoint ships in `data/Policies/Go2/`.

Changes apply the next time the example is Loaded. Each field has a **Reset** button
to go back to the shipped default.

## ROS2 bridge (optional, for nav2)

Turn on **Edit > Preferences > Go2 Policy Example > ROS2 Bridge > Enable ROS2 Bridge**,
then **Load** the example again. This builds an OmniGraph (`/World/Go2ROS2`, using the
same node types as `isaacsim.ros2.bridge`'s own "Extensions > ROS2" graph shortcuts)
that:

- **Subscribes** to `cmd_vel` (`geometry_msgs/Twist`) and drives the robot with it.
  The keyboard still works and takes priority while a mapped key is held; release it
  and control falls back to the latest `cmd_vel`. This is what lets nav2's controller
  server drive the robot once a Nav2 stack is pointed at this stage.
- **Publishes** `odom` (`nav_msgs/Odometry`, ground-truth chassis pose/twist), the
  `world -> odom -> <chassis frame>` TF chain plus the robot's own link tree (`tf`),
  `joint_states`, and (optionally) `/clock`.

Topic names, node namespace, ROS domain ID, and the chassis frame id are all fields in
that same Preferences section -- nothing is hardcoded, so multiple robots can run side
by side with different namespaces/domain IDs. Odometry/TF use the robot's articulation
root as the reference prim, so this keeps working regardless of how a swapped-in
`go2.usd` names its internal links; only the frame-id *strings* (cosmetic, for
RViz/nav2) need to match your actual USD if you care about that naming.

For an actual nav2 stack: run nav2 as a normal ROS2 process (outside Isaac Sim) with
`use_sim_time:=true`, and make sure its `ROS_DOMAIN_ID`/RMW settings match this
extension's (or your system's ROS2 environment, if `ros2_domain_id` is left empty).

### rclpy fails to import inside Isaac Sim ("Could not import system/internal rclpy")

If Isaac Sim's own log shows `Could not import system rclpy` / `Could not import
internal rclpy` and nothing this extension publishes shows up in `ros2 topic list`,
the terminal Isaac Sim was launched from almost certainly has a *system* ROS2
install sourced (e.g. `source /opt/ros/humble/setup.bash` in `.bashrc`), which
pollutes `PYTHONPATH`/`AMENT_PREFIX_PATH` with a Python-3.10-built `rclpy` --
incompatible with Isaac Sim's embedded Python 3.11, so it fails to load and the
ROS2 bridge silently never actually initializes (no publishers/subscribers get
created at all, independent of domain ID). Fix: launch Isaac Sim from a terminal
with `ROS_VERSION`, `ROS_PYTHON_VERSION`, `ROS_DISTRO`, `AMENT_PREFIX_PATH`,
`COLCON_PREFIX_PATH`, `PYTHONPATH`, and `CMAKE_PREFIX_PATH` unset, so it falls back
to its own bundled, Python-3.11-matched internal `rclpy` -- the `ros2` CLI/nav2 in
your normal ROS2 terminal are unaffected (DDS discovery doesn't care about Python
versions), as long as `ROS_DOMAIN_ID`/`RMW_IMPLEMENTATION` still match.

A convenient way to do this every time: add a small launcher function to your
`.bashrc`/`.zshrc` that strips those variables right before starting Isaac Sim,
and a one-word alias for this extension specifically:

```bash
isaac_run() {
  # Unset ROS2 env vars so Isaac Sim's own bundled (Python-3.11-matched) rclpy
  # loads instead of a system ROS2 install's (usually Python-3.10-built) one.
  unset ROS_VERSION ROS_PYTHON_VERSION ROS_DISTRO AMENT_PREFIX_PATH \
        COLCON_PREFIX_PATH PYTHONPATH CMAKE_PREFIX_PATH
  "$HOME/isaacsim/isaac-sim.sh" "$@"
}

# Launches Isaac Sim with this extension enabled, via the clean launcher above.
alias isaac_go2_run='isaac_run --enable go2_in_isaacsim'
```

(Adjust `$HOME/isaacsim` to your actual Isaac Sim install path.) After `source
~/.bashrc`, `isaac_go2_run` opens Isaac Sim with `go2_in_isaacsim` enabled and a
working ROS2 bridge, without needing to remember the `unset`/`--enable` incantation
each time.

## Mid-360 lidar (optional)

Unlike the ROS2 bridge above, this isn't a Preferences toggle: it's a **separate
Robot USD**, `data/Robots/Go2/usd/go2_with_mid360.usd`, alongside the bare
`go2.usd`. Pick **Go2 with Mid-360** from the **Preset** dropdown under **Robot
USD** in Preferences (or type/browse its path), then **Load** the example again.
What you see is what's actually sensing -- the same prim carries the visual mesh
and the lidar API, not an invisible sensor mounted separately in Python.

`go2_with_mid360.usd` references `go2.usd` plus `data/Sensors/Mid360/Mid360.usd`
(a real Mid-360 CAD model, converted from a STEP file to USD), mounted on
`base` at an approximate head-top offset (measure your actual bracket and rebuild
if you need it exact -- see `go2_in_isaacsim/mid360.py`'s module docstring for how
this file was built). An RTX Lidar sensor API is applied to a small `Camera` prim
alongside the visual mesh, using a custom scan profile
(`data/lidar_configs/Livox/Mid360.json`) approximating the real Mid-360's headline
spec -- 360deg horizontal x -7..+52deg vertical FOV, ~40m range, ~200,000
points/sec, 905nm -- since Isaac Sim doesn't ship an official Mid-360 profile (only
Velodyne/Ouster/Hesai/SICK/etc.). It's built from 40 evenly-spaced vertical
channels swept through a full rotation, so it matches the Mid-360's FOV/range
envelope but **not** its actual non-repetitive (rosette) scan pattern.

If **ROS2 Bridge** is also enabled and the loaded Robot USD has a Mid-360 (found
by scanning for the RTX Lidar API, so this works on any Robot USD that has one,
not just the bundled one), its point cloud is published as
`sensor_msgs/PointCloud2` on **PointCloud2 Topic** (default `livox/lidar`, matching
the real `livox_ros_driver2`'s default topic), with a TF from the chassis frame to
**Frame Id** (default `livox_frame`) computed from the sensor prim's *actual*
transform in the USD -- not a guessed offset -- so it's always correct regardless
of which Robot USD (or mount position) is loaded. If the loaded Robot USD has no
Mid-360 (e.g. bare `go2.usd`), this is silently skipped.

## Notes for redistribution

- The bundled default checkpoint came from
  `unitree_rl_lab/logs/rsl_rl/unitree_go2_velocity/2026-02-17_22-16-47`
  (`exported/policy.pt` + `params/env.yaml` + `params/deploy.yaml`).
- The observation vector and action scale/clip in `go2.py` are built at runtime from
  the checkpoint's `params/deploy.yaml` (term list/order/scale), not hardcoded -- see
  `Go2FlatTerrainPolicy._compute_observation` and `_OBSERVATION_TERMS`. Swapping in a
  policy trained with a different (proprioceptive) observation set works by just
  pointing **Policy deploy.yaml** at the new checkpoint; a term this file doesn't know
  how to compute (e.g. `height_scan`, which needs a sensor) raises `NotImplementedError`
  naming the missing term instead of silently producing a wrong observation vector.
