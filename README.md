# go2_in_isaacsim

A pure-Isaac-Sim Isaac Sim extension that runs a [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab)
(Isaac Lab)-trained Unitree Go2 locomotion policy, in the same style as the
built-in **Isaac Examples > Policy > Spot** example — click Load, click Play,
drive with the keyboard. No standalone Python script to run.

See [`docs/README.md`](docs/README.md) for what's inside and how to configure it
(robot USD, environment USD, policy checkpoint — all editable from
`Edit > Preferences > Go2 Policy Example`, no code changes needed).

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

## What's bundled

- `data/Robots/Go2/usd/` — Go2 USD, sourced from
  [unitree_model](https://github.com/unitreerobotics/unitree_model)'s
  `Go2/usd/` (converted from Unitree's URDF). Not modified.
- `data/Policies/Go2/` — a `unitree_rl_lab`-trained flat-terrain velocity
  policy (`policy.pt` + the matching Isaac Lab `params/env.yaml`).

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
