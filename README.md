# fm-robot-agent

The host-side control agent that makes a robot a First Motive device.

## What

One agent runs on each robot host, outside any container, and serves a fixed
verb set over Zenoh:

```
status · up · down · config · record · stop · episodes · session · collect
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
router. Add `--dry-run` to see what it would write. The card is read from
`/etc/fm/machine.json` on Linux and `~/.config/fm/machine.json` on macOS, the
same two paths fm-comms resolves.

### On an operator machine

The office Mac mini that hosts the router, and any Mac driving the fleet over
the tailnet, run the client half and nothing else. There is no unit to install
here, and `install.sh` refuses such a host on its own terms: its card declares
role `mac`, not `robot`.

Three things make `fm robot` work on one:

- **The repo, cloned into the fm workspace.** `fm` reads `fm.json` there and
  mounts the `robot` verb; `fm setup` is what puts it there.
- **`uv` on `PATH`.** `scripts/run/robot.sh` runs the client through
  `uv run --project`, which resolves the project on first use.
- **The router endpoint.** On the Mac mini it comes from fm-comms'
  `/etc/fm-comms.env`, which the client reads directly — no unit runs there to
  put it in the environment first. A Mac without fm-comms exports it instead:

```bash
export FM_ROUTER_ENDPOINT=tcp/<router-tailnet-address>:7447
```

A client session is opened per command and closed with it, so an operator
machine holds no state and runs no service.

### Hosting a robot with no computer

An SO-101 leader/follower pair has no computer of its own: each arm is a
Feetech bus behind a USB serial board. Whichever tailnet machine the arms are
plugged into hosts the robot, and keeps its own identity while it does:

```bash
# once per host: write the robot's profile (ports by USB serial number)
fm robot host fm-rob-03 \
  --leader-port serial:<leader board> --follower-port serial:<follower board> \
  --leader-id fm_rob_03_leader --follower-id fm_rob_03_follower \
  --root <workspace>/data/hf/lerobot/fm-rob-03 \
  --stack-project <workspace>/fm-teleop/fm_teleop_so101

# every time after: plug the arms in and host it
fm robot host fm-rob-03
```

The agent serves `fm/robot/fm_rob_03/*` from this machine and starts fm-teleop's
`fm-teleop-so101` stack, which owns both arms. `fm robot fm-rob-03 ...` and the
Desktop reach it from anywhere on the tailnet while it runs; stopping the
command (Ctrl-C) ends the stack, turns the follower's torque off, and the robot
leaves the fabric. The profile lives in `~/.config/fm/robots/` on macOS and
`/etc/fm/robots/` on Linux; it holds no secret, but it is per host, not source.

Datasets land under the profile's `root`. With `<workspace>/data/hf/lerobot/<name>`
there, `fm policy train <name>/<dataset>` on the same machine finds them.

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
fm robot fm-rob-02 record start --dataset checkers-bag-v1 --task "put the nuts in the bag"
fm robot fm-rob-01 collect start --object can --hours 2
fm robot fm-rob-01 collect status
fm robot fm-rob-01 collect stop
fm robot fm-rob-01 stop
```

`list` is a wildcard query, so discovering a robot takes no hostname and no
port. A robot that answers is online by definition, and every reply carries the
`device` it came from — the agent answers under its own key, never the selector
that asked, so a wildcard reply names the robot behind it. `mode` is sugar over
`config set` of the one key each robot spells its own way.

`--task` is the instruction the episode demonstrates, which a
language-conditioned policy trains on. A robot whose recorder has no field for
one refuses rather than dropping the sentence: record without it, then write the
text onto the episodes with `fm policy dataset relabel`. Neither recorder takes
one today.

`collect` drives the Anvil's unattended visuo-tactile collection loop through
`scripts/run/tactile-collect.sh` in the anvil-embodied-ai checkout
(`FM_ANVIL_EMBODIED_AI_DIR`, default `~/anvil-embodied-ai`). The loop moves the
arm unattended, so a start is refused until an operator arms it on the robot
itself:

```bash
touch ~/.local/state/fm-robot-agent/collect-start-enabled   # on the robot; rm to disarm
```

The flag lives in the agent's state directory (`FM_ROBOT_AGENT_STATE_DIR`) and
is read on every start, so neither step needs a restart. `stop` and `status`
are always answered, since stop is the safety path.

`start` takes `--object` and optionally `--hours` (0.01 to 12), `--cycles`
(1 to 10000), `--speed` (0.1 to 3.0, the fastest verified on hardware;
the loop defaults to 2.5) and `--no-record`. The robot's own preflight refuses a start it
cannot supervise, and the client prints a `warning:` line for each condition a
start went ahead through. `stop` answers once the arm is home, which can take
three minutes, and the agent refuses a second collect call while one runs.
`status` is the default. The other robots refuse the verb.

A severing write answers only once the robot has watched its own telemetry come
back, which takes the stack's recreate plus the bridge's discovery — up to 90
seconds. If the telemetry stays dead, the robot restores both files, restarts
both units, and the command reports the revert.

## Anvil Upkeep

On the Anvil the agent runs one upkeep task every 30 seconds. It turns off the
vendor replay buffer while `.env.config` has `ENABLE_CYCLONEDDS=true`. Anvil's
Known Issues say the buffer leaks memory in that configuration. On fm-rob-01 it
grew about 600 MB a minute, and the kernel killed it during a take on
29 September 2026. The switch is volatile: a stack restart turns the buffer on
again, and the next tick turns it off. To keep the buffer for a debugging
session (the webapp's Diagnostics → Save Buffer), set
`FM_ANVIL_KEEP_REPLAY_BUFFER=1` in the agent's environment.

`status` reports what an operator needs to see this coming:

- `memory`: the host's `total_kb`, `available_kb`, `swap_total_kb` and
  `swap_free_kb` from `/proc/meminfo`, or `null` where the agent is not on the
  robot's host (the Axol).
- `replay_buffer`: `enabled`, and `held_off` when the agent keeps it off.
- `capture.quest_metrics`: the headset's live controller rates. The
  controller-tracking entries in `capture.quest` are start-up checks; they stay
  failed while teleop works.

## Status

This repo was seeded from the HTTP agent running on the Anvil workcell devbox,
kept under `agent/` as the baseline. The Zenoh port replaces it.

## Safety

The agent exposes no motion topic and subscribes to no command topic. `stop`
maps to the robot vendor's own pause or disconnect, never to anything this repo
invents.

The fabric never sends a pose, trajectory or command topic. It can start and
stop supervised behaviours that move the arm: `collect` on the Anvil, only once
armed locally and behind the robot's own preflight, and the Axol's
`run-policy`.

No configuration key commands motion either. The keys that shape how the arms
move are refused unless the robot reports itself idle — a compose stack that is
down on the Anvil, no running operation on the Axol — and idle is read off the
robot rather than asserted by the caller.

## Development

See `CONTRIBUTING.md` for the branch, commit, and pull request workflow.
