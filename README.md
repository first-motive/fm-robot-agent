# fm-robot-agent

The host-side control agent that makes a robot a First Motive device.

## What

One agent runs on each robot host, outside any container, and serves a fixed
verb set over Zenoh:

```
status · up · down · config · record · stop · episodes
```

Every byte reaches the fleet through the Zenoh router on 7447, so a robot is
reachable from the office LAN or from anywhere on the tailnet without opening a
second port. `fm robot <name> <verb>` and the fm-desktop app are the two faces
of the same verb set.

The agent owns the operations no node inside the robot's own graph can perform:
rewriting the robot's own configuration, restarting the stack it would otherwise
die with, and reporting the filesystem facts behind episodes and disk space.

`config` reads and writes every key in that configuration — the Anvil's
`.env.config`, the Axol's Almond settings — and every key carries a class that
decides its guard:

| Class | Guard |
| --- | --- |
| `severing` | Written, restarted, verified against the robot's own telemetry, reverted if it does not come back |
| `motion` | Refused unless the robot reports itself idle |
| `tuning` | Written through |
| `mode` | Validated against what the robot actually offers |
| `unknown` | Readable, never writable |

An unclassified key is never written because `.env.config` is an `env_file` for a
container that runs `privileged: true`: writing an unknown key there is
environment injection, not a configuration edit.

## Install

On the robot's own host, once its identity card and fm-comms are in place:

```bash
./install.sh --role anvil     # the Anvil workcell devbox
./install.sh --role axol      # the Almond Axol
```

The installer refuses rather than guesses: the card must declare role `robot`
with a `robot` kind matching the role given, and `/etc/fm-comms.env` must name a
router. Add `--dry-run` to see what it would write.

## Use

```bash
fm robot list                                        every robot on the fabric
fm robot fm-rob-01 status --json
fm robot fm-rob-01 up
fm robot fm-rob-01 mode openarm_v2_quest_teleop.yaml
fm robot fm-rob-01 config get
fm robot fm-rob-01 config set CYCLONEDDS_VERBOSITY=fine
fm robot fm-rob-01 config rollback
fm robot fm-rob-01 record start --dataset grocery-sort-v1
fm robot fm-rob-01 stop
```

`list` is a wildcard query, so discovering a robot takes no hostname and no
port. A robot that answers is online by definition. `mode` is sugar over
`config set` of the one key each robot spells its own way.

A severing write answers only once the robot has watched its own telemetry come
back, which takes the stack's recreate plus the bridge's discovery — up to 90
seconds. If the telemetry stays dead, the robot restores both files, restarts
both units, and the command reports the revert.

## Status

This repo was seeded from the HTTP agent running on the Anvil workcell devbox,
kept under `agent/` as the baseline. The Zenoh port replaces it.

## Safety

The agent exposes no motion topic and subscribes to no command topic. `stop`
maps to the robot vendor's own pause or disconnect, never to anything this repo
invents.

No configuration key commands motion either. The keys that shape how the arms
move are refused unless the robot reports itself idle — a compose stack that is
down on the Anvil, no running operation on the Axol — and idle is read off the
robot rather than asserted by the caller.

## Development

See `CONTRIBUTING.md` for the branch, commit, and pull request workflow.
