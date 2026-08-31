# fm-anvil-agent

A small host-side control agent for the Anvil workcell, added by First Motive
on the `fm-agent` branch of this fork. It gives the fm-desktop app an HTTP API
for the operations that cannot ride any bridge node inside the ROS graph:
switching the control mode (which recreates every container, including the
bridges) and starting or stopping the compose stack. It also serves the
filesystem facts the app previously read over SSH.

## Install (Once Per Rig)

```bash
cd ~/anvil-loader
git fetch fm && git checkout fm-agent   # this branch: upstream 1.2.4 + agent/
sudo bash agent/install.sh
```

The service runs as user `anvil` (docker group), listens on `:8770`, and
restarts automatically. Logs: `journalctl -u fm-anvil-agent`. The installer
also drops an avahi advert (`_fm-rig._tcp`, `role=anvil`) so fm-desktop's
Settings discovers the rig on the LAN.

## API

| Endpoint | Does |
|---|---|
| `GET /health` | liveness + version |
| `GET /state` | current mode, available configs, compose services, disk |
| `GET /datasets` | dataset directories under `data/recordings/` |
| `GET /episodes?dataset=<slug>` | episode dirs with size + raw `metadata.yaml` |
| `POST /mode {"config": "...yaml", "apply": true}` | rewrite `ARMS_CONTROL_CONFIG_FILE`, optionally recreate the stack |
| `POST /stack {"action": "up"\|"down"\|"recreate"}` | compose verb, detached |
| `POST /episodes/delete {"dataset", "slug"}` | remove one episode through the ros2 container (root-owned files) |

Compose operations detach (own session, log at
`/tmp/fm-anvil-agent-compose.log`); poll `GET /state` to watch them land.

## Security

Fixed verbs only; every input is validated against a strict pattern or the
real config directory; subprocesses use argument lists, never a shell. The
port is as open as the rig's other LAN services (rosbridge :9090 can already
e-stop and record without auth) — keep it on the workcell LAN.

## Branch Policy

`fm-agent` is based on the exact upstream commit the rig runs (v1.2.4,
`5611ee2`) so deploying it never changes the pinned compose images. To move to
a newer anvil-loader release: rebase this branch onto the new upstream tag,
test, then update the rig checkout.
