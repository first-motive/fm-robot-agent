# fm-robot-agent

The host-side control agent that makes a robot a First Motive device.

## What

One agent runs on each robot host, outside any container, and serves a fixed
verb set over Zenoh:

```
status · up · down · mode · record · stop · episodes
```

Every byte reaches the fleet through the Zenoh router on 7447, so a robot is
reachable from the office LAN or from anywhere on the tailnet without opening a
second port. `fm robot <name> <verb>` and the fm-desktop app are the two faces
of the same verb set.

The agent owns the operations no node inside the robot's own graph can perform:
rewriting the control-mode config, restarting the stack it would otherwise die
with, and reporting the filesystem facts behind episodes and disk space.

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
fm robot fm-rob-01 record start --dataset grocery-sort-v1
fm robot fm-rob-01 stop
```

`list` is a wildcard query, so discovering a robot takes no hostname and no
port. A robot that answers is online by definition.

## Status

This repo was seeded from the HTTP agent running on the Anvil workcell devbox,
kept under `agent/` as the baseline. The Zenoh port replaces it.

## Safety

The agent exposes no motion topic and subscribes to no command topic. `stop`
maps to the robot vendor's own pause or disconnect, never to anything this repo
invents.

## Development

See `CONTRIBUTING.md` for the branch, commit, and pull request workflow.
