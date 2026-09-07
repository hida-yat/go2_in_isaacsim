# Go2 Policy Example

Runs a `unitree_rl_lab`-trained Go2 flat-terrain velocity policy in pure Isaac Sim,
using the same `PolicyController` base class as the built-in Spot Quadruped example
(`isaacsim.robot.policy.examples`).

Open via **Isaac Examples > Policy > Go2**, click **Load**, then **Play**.
Drive with arrow keys / numpad (same mapping as Spot).

## Configuration

Nothing here is hardcoded in Python. Open **Edit > Preferences > Go2 Policy Example**
to point this extension at your own assets:

- **Robot USD** — the Go2 USD to spawn. Defaults to the bare `go2.usd`; point this at a
  variant with sensors + an Action Graph added to bring those sensors along.
- **Environment USD** — optional world/environment USD. Leave empty for the default
  flat ground plane.
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
This example does not publish any exteroceptive sensor (lidar/depth) -- point
**Robot USD** in Preferences at a Go2 USD with sensors + an Action Graph publishing
them (e.g. `sensor_msgs/LaserScan` or `PointCloud2`) for nav2's costmaps; everything
above keeps working unmodified on top of that.

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
