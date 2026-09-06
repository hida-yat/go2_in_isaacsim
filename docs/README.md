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

## Notes for redistribution

- The bundled default checkpoint came from
  `unitree_rl_lab/logs/rsl_rl/unitree_go2_velocity/2026-02-17_22-16-47`
  (`exported/policy.pt` + `params/env.yaml`).
- The observation/action layout in `go2.py` (45-dim obs: ang_vel, gravity, velocity
  commands, joint_pos_rel, joint_vel_rel, last_action; action = JointPositionAction,
  scale 0.25) is specific to that checkpoint's `params/deploy.yaml`. Swapping in a
  policy trained with a different observation set requires updating
  `Go2FlatTerrainPolicy._compute_observation` to match.
