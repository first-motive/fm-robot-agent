"""The Anvil workcell, driven from its host.

Three mechanisms, each for the thing only it can do:

``docker compose``
    Bring the stack up and down, and recreate it after a mode change. The agent
    runs outside the stack precisely so it survives restarting it.

the webapp's tRPC API
    Start and stop recording. The webapp watches ``/recording_status`` and stops
    any recording it did not start itself, so recording cannot be driven any
    other way. It also owns the episode database the Anvil web UI reads.

``ros2 service call`` inside the ros2 container
    ``stop`` and the two state getters. ``anvil_msgs`` lives only inside that
    container, so a service call has to be made from within it.

Mode is a file: ``ARMS_CONTROL_CONFIG_FILE`` in ``.env.config``, validated against
the real ``config/`` directory and applied by recreating the stack.
"""

from __future__ import annotations

import ipaddress
import os
import re
import shlex
import shutil
import subprocess
import sys
import time
from pathlib import Path

from fm_robot_agent.config import (
    MODE,
    MODE_ALIAS,
    MOTION,
    SEVERING,
    TUNING,
    UNKNOWN,
    ConfigError,
    Journal,
    Setting,
    env_replaced,
    env_values,
    restore,
)
from fm_robot_agent.protocol import AdapterError, Outcome
from fm_robot_agent.trpc import TrpcClient

KIND = "anvil-openarm-v2"

MODE_KEY = "ARMS_CONTROL_CONFIG_FILE"
COMPOSE_LOG = Path("/tmp/fm-robot-agent-compose.log")

#: The writer that sets a key in the fleet file, and the account's one way past
#: its root ownership. fm-setup's `robot-sudo` verb installs it and grants this
#: exact path, so the agent can keep the two files in step without holding write
#: access to a file two systemd units read. Absent on a bench host, where the
#: fleet file is usually writable anyway.
FLEET_WRITER = Path(os.environ.get("FM_COMMS_WRITER", "/usr/local/sbin/fm-comms-set"))

#: The fleet's own transport file. The Anvil's `.env.config` sets the domain and
#: the interface its containers use; this file sets the ones the zenoh bridge
#: uses to reach the same graph. They are two spellings of one fact.
FM_COMMS_ENV = Path(os.environ.get("FM_COMMS_ENV", "/etc/fm-comms.env"))

#: What each key in `.env.config` is, and therefore which guard it gets. The
#: table is ours because Anvil ships no schema for that file — see
#: :mod:`fm_robot_agent.config` for what each class means. A key not listed here
#: is reported and never written.
CONFIG_CLASSES = {
    "ROS_DOMAIN_ID": SEVERING,
    "CYCLONEDDS_IFACE": SEVERING,
    "CYCLONEDDS_TRANSPORT": SEVERING,
    "ENABLE_CYCLONEDDS": SEVERING,
    "CYCLONEDDS_ALLOW_MULTICAST": SEVERING,
    "TELEOP_POSITION_SCALE": MOTION,
    "ENABLE_VR_TELEOP": MOTION,
    "CYCLONEDDS_PEER_IP": TUNING,
    "CYCLONEDDS_FRAGMENT_SIZE": TUNING,
    "CYCLONEDDS_MAX_MESSAGE_SIZE": TUNING,
    "CYCLONEDDS_WHC_HIGH": TUNING,
    "CYCLONEDDS_VERBOSITY": TUNING,
    MODE_KEY: MODE,
}

#: Keys the fleet file spells differently. Writing one file and not the other is
#: exactly the defect the 2026-09-01 run found twice: a bridge on `docker0` while
#: the stack was on `wlp8s0`, and a bridge on the fleet's domain while the stack
#: was on the robot's. Both reported success and carried nothing.
PAIRED_KEYS = {
    "ROS_DOMAIN_ID": ("FM_ROS_DOMAIN_ID", "ROS_DOMAIN_ID"),
    "CYCLONEDDS_IFACE": ("FM_DDS_IFACE",),
}

#: What CycloneDDS accepts for its own verbosity, in its own spelling.
VERBOSITY_LEVELS = ("finest", "finer", "fine", "config", "info", "warning", "severe", "none")

TRANSPORT_MODES = ("udp", "tcp")

BOOLEAN_VALUES = ("true", "false")

#: An interface name as the kernel limits it: IFNAMSIZ is 16 bytes including the
#: terminator, and nothing else is a legal name.
IFACE_PATTERN = re.compile(r"^[A-Za-z0-9._-]{1,15}$")

#: The highest DDS domain the RTPS port mapping leaves room for.
MAX_DOMAIN_ID = 232

#: Where the open severing change is journalled. Not in the loader directory:
#: that directory is Anvil's, and a file of ours in it is one more thing their
#: next release has to survive.
STATE_DIR = Path(
    os.environ.get("FM_ROBOT_AGENT_STATE_DIR", str(Path.home() / ".local" / "state" / "fm-robot-agent"))
)

#: The unit that carries this robot's telemetry onto the fabric. Restarted after
#: a severing write because it reads the same two values from the fleet file.
BRIDGE_UNIT = os.environ.get("FM_BRIDGE_UNIT", "fm-zenoh-bridge")

#: The topic whose arrival means the data plane came back. It is the one every
#: rig publishes and the desktop subscribes to, so if this moves nothing else the
#: fleet watches is moving either.
TELEMETRY_TOPIC = "/joint_states"

#: How long a severing change is given to prove itself, and how often it is
#: asked. Ninety seconds is the stack's own recreate time plus the bridge's
#: discovery, measured on the workcell rather than guessed.
VERIFY_WINDOW_S = 90.0
VERIFY_INTERVAL_S = 5.0

#: How long one probe waits for a message. Shorter than the interval so a probe
#: that finds nothing still leaves time for the next one.
PROBE_TIMEOUT_S = 4

COMPOSE_ACTIONS = {
    "up": ["up", "-d"],
    "down": ["down"],
    "recreate": ["up", "-d", "--force-recreate"],
}

#: Episode directories the recorder writes: a zero-padded counter, nothing else.
SLUG_PATTERN = re.compile(r"^[0-9]{4,}$")

#: ``metadata.yaml`` is written by the recorder, but it sits in a directory an
#: operator can also write to. Reading it unbounded would let one oversized file
#: exhaust the agent's memory, so it is truncated rather than trusted.
METADATA_MAX_BYTES = 1 << 20

#: `ros2 service call` prints a Python repr of the response. Parsing that is
#: tradeoff: brittle if anvil_msgs renames the field, and the alternative is a
#: rclpy client inside a container the agent deliberately runs outside of.
IS_RECORDING_PATTERN = re.compile(r"is_recording=(True|False)")

#: What a service call has to set up before `ros2` will answer, and why each is
#: needed. `docker compose exec` starts a process WITHOUT the image entrypoint,
#: which is the thing that normally sources ROS — so a bare `ros2` is not even on
#: PATH. The workcell's own messages are built into a workspace overlay rather
#: than the base install, so `anvil_msgs` is unknown without the second source.
#: And the image ships CycloneDDS while the default resolution picks Fast DDS,
#: which reaches the service and then fails decoding its reply ("payload size 24
#: is larger than the history payload size of 11").
ROS_SETUP = "/opt/ros/$ROS_DISTRO/setup.bash"
WORKSPACE_SETUP = os.environ.get("FM_ANVIL_ROS_OVERLAY", "/workspace/install/setup.bash")
RMW = os.environ.get("FM_ANVIL_RMW", "rmw_cyclonedds_cpp")

SERVICE_TIMEOUT_S = 15
COMPOSE_TIMEOUT_S = 30


class AnvilAdapter:
    """One Anvil workcell, reached through the loader directory on this host."""

    kind = KIND

    def __init__(
        self,
        loader_dir: Path | None = None,
        webapp_url: str | None = None,
        comms_env: Path | None = None,
        journal: Journal | None = None,
    ) -> None:
        self.loader_dir = loader_dir or Path(
            os.environ.get("FM_ANVIL_LOADER_DIR", str(Path.home() / "anvil-loader"))
        )
        self.config_dir = self.loader_dir / "config"
        self.env_config = self.loader_dir / ".env.config"
        self.recordings_dir = self.loader_dir / "data" / "recordings"
        self.comms_env = comms_env or FM_COMMS_ENV
        self.journal = journal or Journal(STATE_DIR / "config-journal.json")
        # The webapp is on this host. Recording is a localhost call, never a
        # fleet one: the fabric reaches the agent, and the agent reaches the
        # webapp, which is what keeps port 3000 off the tailnet.
        self.webapp = TrpcClient(webapp_url or "ws://localhost:3000/trpc")

    # --- the verb set --------------------------------------------------------

    def status(self) -> dict:
        return {
            "mode": self.read_mode(),
            "modes": self.list_configs(),
            # The live controller state is publish-on-change with no getter
            # service, so it reaches the desktop over `<ns>/hardware_state_
            # controller/state` on the fabric. What the agent can answer for is
            # whether the stack that would publish it is running at all.
            "hardware": "running" if self._stack_running() else "down",
            "recording": self._is_recording(),
            "services": self.compose_ps(),
            "disk": self.disk(),
        }

    def up(self) -> Outcome:
        return self._compose_detached("up")

    def down(self) -> Outcome:
        return self._compose_detached("down")

    def set_mode(self, config: str) -> Outcome:
        """Sugar over the config verb, kept because `fm robot X mode Y` predates it."""
        return self.config_write(MODE_ALIAS, config)

    def record(self, dataset: str, action: str) -> Outcome:
        if action == "start":
            session = self._find_or_create_session(dataset)
            episode_id = self.webapp.call(
                "mutation", "recording.start", {"sessionId": session["id"]}
            )
            return Outcome(
                ok=True,
                message=f"recording into {dataset}",
                detail={"episode": str((episode_id or {}).get("episodeId", ""))},
            )
        stopped = self.webapp.call("mutation", "recording.stop", {"note": "stopped from the fleet"})
        return Outcome(
            ok=True,
            message="recording stopped",
            detail={"episode": str((stopped or {}).get("id", ""))},
        )

    def stop(self) -> Outcome:
        """Pause the hardware state controller — the workcell's own halt.

        Not an emergency stop. No e-stop exists on this robot, and the `estop`
        state the controller also accepts is the operator's physical procedure,
        not something the fleet reaches in over a network.
        """
        self._service_call(
            "/hardware_state_controller/set_state",
            "anvil_msgs/srv/SetHardwareState",
            "{state: pause}",
        )
        return Outcome(ok=True, message="hardware paused", detail={"state": "pause"})

    def episodes(self, dataset: str) -> list[dict]:
        base = self.recordings_dir / dataset
        try:
            slugs = sorted(entry.name for entry in base.iterdir())
        except OSError:
            return []
        found = []
        for slug in slugs:
            path = base / slug
            if not SLUG_PATTERN.match(slug) or not path.is_dir():
                continue
            found.append(
                {
                    "slug": slug,
                    "size_kb": _dir_size_kb(path),
                    "metadata_yaml": _read_capped(path / "metadata.yaml"),
                }
            )
        return found

    # --- config --------------------------------------------------------------

    @staticmethod
    def _read(path: Path) -> str:
        """A config file's text, or empty when it is not there yet."""
        try:
            return path.read_text(encoding="utf-8")
        except OSError:
            return ""

    def config_read(self) -> list[Setting]:
        """Every key in this robot's own `.env.config`, classified by our table.

        The file is read rather than a key list held here: Anvil version-controls
        it and edits it about once per release, and a hardcoded list would go
        stale without anything saying so. A key we have not classified is still
        reported — an operator who cannot see a key cannot ask about it — and is
        marked unwritable.
        """
        values = env_values(self._read(self.env_config))
        settings = []
        for key, value in values.items():
            klass = CONFIG_CLASSES.get(key, UNKNOWN)
            settings.append(
                Setting(
                    key=key,
                    value=value,
                    klass=klass,
                    options=tuple(self.list_configs()) if klass == MODE else _options(key),
                )
            )
        return sorted(settings, key=lambda setting: setting.key)

    def config_write(self, key: str, value: str) -> Outcome:
        """Validate one key, check its guard, and write it to every file that holds it."""
        if key == MODE_ALIAS:
            key = MODE_KEY
        klass = CONFIG_CLASSES.get(key, UNKNOWN)
        if klass == UNKNOWN:
            return Outcome(
                ok=False,
                message=f"{key} is not a key this agent classifies, so it is read-only",
                detail={"key": key, "class": UNKNOWN},
            )
        try:
            value = self._validated(key, value)
        except ConfigError as exc:
            return Outcome(ok=False, message=str(exc), detail={"key": key, "class": klass})
        if klass == MOTION and self._stack_running():
            return Outcome(
                ok=False,
                message=f"{key} shapes motion and the stack is up; take it down first",
                detail={"key": key, "class": klass},
            )
        if klass == SEVERING:
            return self._write_severing(key, value)
        self._write_paired(key, value)
        applied = self._apply(klass)
        return Outcome(
            ok=True,
            message=f"{key}={value}; {applied}",
            detail={"key": key, "value": value, "class": klass},
        )

    def _write_severing(self, key: str, value: str) -> Outcome:
        """Write a transport key, then prove the robot still reaches the fleet.

        The control plane this reply travels on is zenoh over TCP to the router
        and no transport key touches it, so the agent stays reachable however
        wrong the value was. What a wrong value takes away is the telemetry — and
        it takes it away silently, which is why this waits for the data plane
        rather than for an error nobody will get.
        """
        if self.journal.read() is not None:
            return Outcome(
                ok=False,
                message="a severing change is already open; roll it back first",
            )
        previous = self._paired_previous(key, value)
        self.journal.open(key, value, previous)
        try:
            self._write_paired(key, value)
        except AdapterError:
            # Nothing was left changed, so nothing is left to undo — and a
            # journal left open would refuse every severing write after it.
            self.journal.close()
            raise
        self._restart_data_plane()
        if self._data_plane_returns():
            self.journal.close()
            return Outcome(
                ok=True,
                message=f"{key}={value}; telemetry returned",
                detail={"key": key, "value": value, "class": SEVERING, "verified": True},
            )
        reverted = self._revert(previous, key)
        return Outcome(
            ok=False,
            message=f"{key}={value} killed telemetry; reverted {', '.join(reverted)}",
            detail={"key": key, "value": value, "class": SEVERING, "verified": False},
        )

    def finish_open_change(self) -> Outcome | None:
        """Undo a severing change this agent did not live to verify.

        The journal is closed only once telemetry has been seen, so an agent that
        starts and finds one open is an agent whose predecessor died inside the
        window. Whether it died because of the change or beside it, the honest
        move is the same: put back what was there and say so.
        """
        entry = self.journal.read()
        if entry is None:
            return None
        return self.config_rollback()

    def config_rollback(self) -> Outcome:
        """Restore the files the open severing change replaced, and restart."""
        entry = self.journal.read()
        if entry is None:
            return Outcome(ok=False, message="no change is open")
        restored = self._restore_files(entry["files"], str(entry.get("key") or ""))
        self._restart_data_plane()
        self.journal.close()
        return Outcome(
            ok=True,
            message=f"{entry.get('key', 'the open change')} rolled back; restored {', '.join(restored)}",
            detail={"key": entry.get("key", ""), "restored": restored},
        )

    def _paired_previous(self, key: str, value: str) -> dict:
        """What every file this write touches holds right now."""
        return {path: self._read(path) for path in self._paired_edits(key, value)}

    def _revert(self, previous: dict, key: str = "") -> list[str]:
        restored = self._restore_files(
            {str(path): text for path, text in previous.items()}, key
        )
        self._restart_data_plane()
        self.journal.close()
        return restored

    def _restore_files(self, files: dict, key: str = "") -> list[str]:
        """Put journalled contents back, through the writer where root owns them.

        The plain restore writes files directly and skips what it cannot write,
        which on a robot would leave the fleet file holding the value that killed
        telemetry while the loader's file went back — the half state a revert
        exists to prevent. So a file this process cannot replace is restored key
        by key instead, from the text the journal kept. Only the key that changed
        is put back: rewriting the others to values they already hold would spend
        a sudo call each to change nothing.
        """
        direct = {path: text for path, text in files.items() if _writable(Path(path))}
        restored = restore(direct)
        for path, text in files.items():
            if path in direct:
                continue
            held = env_values(text)
            for changed, aliases in PAIRED_KEYS.items():
                if key and changed != key:
                    continue
                for alias in aliases:
                    if alias not in held:
                        continue
                    try:
                        self._fleet_write(changed, held[alias])
                    except AdapterError as exc:
                        # Reported, not raised: a revert that stops at the first
                        # failure leaves more behind than one that carries on.
                        print(f"fm-robot-agent: {exc}", file=sys.stderr, flush=True)
                    else:
                        restored.append(path)
                    break
        return restored

    def _restart_data_plane(self) -> None:
        """Recreate the stack and restart the bridge, so both read the new values.

        Both, because the pair is the point: the containers take the domain and
        the interface from `.env.config`, and the bridge takes the same two from
        the fleet file. Restarting one of them is how they were last found
        disagreeing.
        """
        self._compose_detached("recreate")
        try:
            subprocess.run(
                ["systemctl", "restart", BRIDGE_UNIT],
                capture_output=True,
                text=True,
                timeout=COMPOSE_TIMEOUT_S,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            # A host without the bridge unit is a bench host, and a bench host
            # has no fabric to lose. The verification below decides either way.
            pass

    def _data_plane_returns(self) -> bool:
        """Wait for telemetry, up to the window a recreate and discovery need."""
        deadline = time.monotonic() + VERIFY_WINDOW_S
        while time.monotonic() < deadline:
            if self._telemetry_arrives():
                return True
            time.sleep(VERIFY_INTERVAL_S)
        return self._telemetry_arrives()

    def _telemetry_arrives(self) -> bool:
        """Whether one message lands on the topic the fleet watches.

        Probed inside the ros2 container rather than from the fabric: the agent
        holds a zenoh session for the verb set, and every adapter here imports no
        zenoh precisely so the suite runs without a router. So this proves the
        graph the bridge subscribes to is alive, not the hop past it — the bridge
        is restarted alongside and its own unit failing is the other half.
        """
        try:
            self._ros_command(
                f"ros2 topic echo --once --timeout {PROBE_TIMEOUT_S} {TELEMETRY_TOPIC}",
                TELEMETRY_TOPIC,
                timeout_s=PROBE_TIMEOUT_S * 2,
            )
        except AdapterError:
            return False
        return True

    def _validated(self, key: str, value: str) -> str:
        """The value as the file should hold it, or raise the reason it may not.

        Validation is per key rather than per class: what makes a domain id wrong
        and what makes an interface name wrong have nothing in common, and a
        transport value that reaches the file unchecked is a robot that comes back
        up carrying nothing.
        """
        value = value.strip()
        if key == MODE_KEY:
            if value not in self.list_configs():
                return _refuse_value(f"unknown config {value!r}; this robot offers {self.list_configs()}")
            return value
        if key == "ROS_DOMAIN_ID":
            return _integer(key, value, low=0, high=MAX_DOMAIN_ID)
        if key == "CYCLONEDDS_IFACE":
            if not IFACE_PATTERN.match(value):
                return _refuse_value(f"{value!r} is not an interface name")
            return value
        if key == "CYCLONEDDS_PEER_IP":
            try:
                return str(ipaddress.ip_address(value))
            except ValueError:
                return _refuse_value(f"{value!r} is not an IP address")
        if key in _BOOLEAN_KEYS:
            lowered = value.lower()
            if lowered not in BOOLEAN_VALUES:
                return _refuse_value(f"{key} is one of {BOOLEAN_VALUES}")
            return lowered
        if key == "CYCLONEDDS_TRANSPORT":
            lowered = value.lower()
            if lowered not in TRANSPORT_MODES:
                return _refuse_value(f"{key} is one of {TRANSPORT_MODES}")
            return lowered
        if key == "CYCLONEDDS_VERBOSITY":
            lowered = value.lower()
            if lowered not in VERBOSITY_LEVELS:
                return _refuse_value(f"{key} is one of {VERBOSITY_LEVELS}")
            return lowered
        if key == "TELEOP_POSITION_SCALE":
            try:
                scale = float(value)
            except ValueError:
                return _refuse_value(f"{key} is a number")
            if not 0 < scale <= MAX_TELEOP_SCALE:
                return _refuse_value(f"{key} is between 0 and {MAX_TELEOP_SCALE}")
            return value
        # The remaining tuning keys are byte counts CycloneDDS reads as integers.
        return _integer(key, value, low=1, high=None)

    def _paired_edits(self, key: str, value: str) -> dict[Path, str]:
        """The new contents of every file this key lives in, keyed by path.

        The fleet file is included only when it exists: a bench host without
        `/etc/fm-comms.env` has no bridge to keep in step, and creating one there
        would invent a fleet the machine is not part of.
        """
        edits = {self.env_config: env_replaced(self._read(self.env_config), key, value)}
        aliases = PAIRED_KEYS.get(key)
        if aliases and self.comms_env.exists():
            text = self._read(self.comms_env)
            for alias in aliases:
                text = env_replaced(text, alias, value)
            edits[self.comms_env] = text
        return edits

    def _write_paired(self, key: str, value: str) -> dict[Path, str]:
        """Write every file this key lives in, or leave all of them unchanged.

        Two files cannot be replaced in one atomic step, so the guarantee is made
        by staging both temporary files first: once both are written, the
        remaining work is two renames within their own directories. A rename that
        still fails puts back what the first one replaced, so the pair is never
        left disagreeing — which is the whole reason it is written together.
        """
        edits = self._paired_edits(key, value)
        previous = {path: self._read(path) for path in edits}
        # The fleet file is root's on a real robot, because two systemd units
        # read it, and the agent runs unprivileged. Where it cannot be written
        # directly it goes through the writer fm-setup's `robot-sudo` installs —
        # which is the only reason a paired write is possible at all there.
        by_hand = {path: text for path, text in edits.items() if _writable(path)}
        delegated = [path for path in edits if path not in by_hand]

        staged = {}
        try:
            for path, text in by_hand.items():
                tmp = path.with_suffix(path.suffix + ".tmp")
                tmp.write_text(text, encoding="utf-8")
                staged[path] = tmp
        except OSError as exc:
            for tmp in staged.values():
                tmp.unlink(missing_ok=True)
            raise AdapterError(f"{key} could not be staged: {exc}") from exc

        replaced = []
        for path, tmp in staged.items():
            try:
                os.replace(tmp, path)
            except OSError as exc:
                for done in replaced:
                    _write_atomic(done, previous[done])
                raise AdapterError(f"{key} could not be written to {path}: {exc}") from exc
            replaced.append(path)

        for path in delegated:
            try:
                self._fleet_write(key, value)
            except AdapterError:
                # Put back what was already written. Half a paired write is the
                # disagreement the pairing exists to make impossible, and a
                # caller seeing the error must be able to trust nothing moved.
                for done in replaced:
                    _write_atomic(done, previous[done])
                raise
        return previous

    def _fleet_write(self, key: str, value: str) -> None:
        """Set a key in the fleet file through the writer, as root.

        One `sudo -n` per alias. It never prompts: where `robot-sudo` has granted
        this path it is silent, and where it has not it fails immediately with
        sudo's own message rather than hanging on a password nobody is there to
        type. The writer decides what the value may be — this passes it on and
        reports what it said.
        """
        for alias in PAIRED_KEYS.get(key, ()):
            try:
                done = subprocess.run(
                    ["sudo", "-n", str(FLEET_WRITER), alias, value],
                    capture_output=True,
                    text=True,
                    timeout=COMPOSE_TIMEOUT_S,
                    check=False,
                )
            except (OSError, subprocess.TimeoutExpired) as exc:
                raise AdapterError(f"{alias} could not be written: {exc}") from exc
            if done.returncode != 0:
                reason = (done.stderr or done.stdout).strip() or f"exit {done.returncode}"
                raise AdapterError(f"{alias} could not be written: {reason}")
            # The domain has two spellings and the writer sets both from either,
            # so the second call would be a no-op that only doubles the sudo.
            return

    def _apply(self, klass: str) -> str:
        """Make the containers see a written key, and say what was done.

        A tuning key is read by a process that already re-reads it, or takes
        effect on the next start; the two classes that change what the containers
        were launched with need the stack recreated.
        """
        if klass in (SEVERING, MODE):
            return self._compose_detached("recreate").message
        return "no restart needed"

    # --- mode ----------------------------------------------------------------

    def list_configs(self) -> list[str]:
        try:
            return sorted(f.name for f in self.config_dir.iterdir() if f.suffix == ".yaml")
        except OSError:
            return []

    def read_mode(self) -> str | None:
        """The current ``ARMS_CONTROL_CONFIG_FILE``; the last assignment wins."""
        value = None
        try:
            lines = self.env_config.read_text(encoding="utf-8").splitlines()
        except OSError:
            return None
        for line in lines:
            stripped = line.strip()
            if stripped.startswith(MODE_KEY + "="):
                value = stripped[len(MODE_KEY) + 1 :].strip()
        return value

    # --- docker compose ------------------------------------------------------

    def compose_ps(self) -> list[dict]:
        """Compose service rows, as ``status`` reports them."""
        import json

        try:
            out = subprocess.run(
                ["docker", "compose", "ps", "--format", "json"],
                cwd=self.loader_dir,
                capture_output=True,
                text=True,
                timeout=COMPOSE_TIMEOUT_S,
                check=False,
            ).stdout
        except (OSError, subprocess.TimeoutExpired):
            return []
        rows = []
        text = out.strip()
        if not text:
            return []
        try:
            parsed = json.loads(text)
            rows = parsed if isinstance(parsed, list) else [parsed]
        except json.JSONDecodeError:
            # Modern compose emits one object per line; older builds a JSON array.
            for line in text.splitlines():
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError:
                    continue
        return [
            {
                "name": row.get("Service") or row.get("Name") or "",
                "state": row.get("State") or "",
                "status": row.get("Status") or "",
            }
            for row in rows
            if row.get("Service") or row.get("Name")
        ]

    def _stack_running(self) -> bool:
        return any(row["state"] == "running" for row in self.compose_ps())

    def _compose_detached(self, action: str) -> Outcome:
        """Run a compose verb in its own session, output to the log.

        Detached so restarting the agent — or restarting the stack the agent's
        caller rides on — can never strand a half-recreated workcell. The caller
        polls ``status`` to watch it land.
        """
        try:
            with COMPOSE_LOG.open("ab") as log:
                subprocess.Popen(
                    ["docker", "compose", *COMPOSE_ACTIONS[action]],
                    cwd=self.loader_dir,
                    stdout=log,
                    stderr=log,
                    start_new_session=True,
                )
        except OSError as exc:
            raise AdapterError(f"docker compose {action} could not start: {exc}") from exc
        return Outcome(ok=True, message=f"compose {action} started", detail={"detached": True})

    # --- ros2 services -------------------------------------------------------

    def _ros_command(self, command: str, what: str, timeout_s: float = SERVICE_TIMEOUT_S) -> str:
        """Run one `ros2` command inside the ros2 container, and return its output.

        ``anvil_msgs`` exists only in that container, so a service call cannot be
        made from the host; the telemetry probe runs there for the same reason.
        Argument list, never a shell — every value in ``command`` is built from
        constants in this module, and no caller-supplied string reaches it.
        """
        # One `bash -c` because the sourcing and the command have to share a
        # shell. The two env-supplied values are quoted like the arguments are.
        # They come from this host's own config rather than the fabric, but a
        # path and an RMW name are still values someone typed, and quoting costs
        # nothing.
        inner = (
            f"source {ROS_SETUP} && source {shlex.quote(WORKSPACE_SETUP)} && "
            f"RMW_IMPLEMENTATION={shlex.quote(RMW)} {command}"
        )
        try:
            result = subprocess.run(
                ["docker", "compose", "exec", "-T", "ros2", "bash", "-c", inner],
                cwd=self.loader_dir,
                capture_output=True,
                text=True,
                timeout=timeout_s,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise AdapterError(f"{what} did not answer: {exc}") from exc
        if result.returncode != 0:
            raise AdapterError(f"{what} failed: {result.stderr.strip() or 'no output'}")
        return result.stdout

    def _service_call(self, service: str, service_type: str, request: str) -> str:
        """Call a ROS service from inside the ros2 container, and return its output."""
        return self._ros_command(
            f"ros2 service call {shlex.quote(service)} {shlex.quote(service_type)} {shlex.quote(request)}",
            service,
        )

    def _is_recording(self) -> bool | None:
        """Whether the workcell is recording, or ``None`` when the stack is down."""
        try:
            output = self._service_call(
                "/get_recording_status", "anvil_msgs/srv/GetRecordingStatus", "{}"
            )
        except AdapterError:
            return None
        match = IS_RECORDING_PATTERN.search(output)
        return match.group(1) == "True" if match else None

    def _record_topics(self) -> list[str]:
        """The workcell's own default record topic set, for a new session."""
        output = self._service_call("/get_topics", "anvil_msgs/srv/GetTopics", "{}")
        return re.findall(r"'(/[^']+)'", output)

    # --- webapp sessions -----------------------------------------------------

    def _find_or_create_session(self, dataset: str) -> dict:
        """The webapp session for a dataset slug, created with the rig's topics when absent."""
        event = self.webapp.first_event("recording.sessions.subscribeAll") or {}
        for session in event.get("sessions", []):
            if session.get("slug") == dataset:
                return session
        created = self.webapp.call(
            "mutation",
            "recording.sessions.create",
            {
                "name": dataset,
                "description": "created by fm-robot-agent",
                "topics": self._record_topics(),
                "slug": dataset,
            },
        )
        if not created:
            raise AdapterError(f"the webapp did not create a session for {dataset!r}")
        return created

    # --- disk ----------------------------------------------------------------

    def disk(self) -> dict | None:
        try:
            usage = shutil.disk_usage(self.loader_dir / "data")
        except OSError:
            return None
        return {"total_kb": usage.total // 1024, "available_kb": usage.free // 1024}


def _dir_size_kb(path: Path) -> int:
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(root, name))
            except OSError:
                continue
    return total // 1024


def _read_capped(path: Path) -> str:
    """Read a file the recorder wrote, truncated to a size the agent can hold."""
    try:
        with path.open(encoding="utf-8", errors="replace") as handle:
            return handle.read(METADATA_MAX_BYTES)
    except OSError:
        return ""


#: Keys whose value is a word docker compose reads as a flag.
_BOOLEAN_KEYS = ("ENABLE_CYCLONEDDS", "CYCLONEDDS_ALLOW_MULTICAST", "ENABLE_VR_TELEOP")

#: The largest teleop scale this refuses past. Not a safety limit — the robot has
#: none of those in software — but a value an operator did not mean to type.
MAX_TELEOP_SCALE = 5.0

#: Values a caller may choose between, for the keys that have a fixed set. The
#: mode key's options come off the robot's own `config/` directory instead.
_OPTIONS = {
    "CYCLONEDDS_VERBOSITY": VERBOSITY_LEVELS,
    "CYCLONEDDS_TRANSPORT": TRANSPORT_MODES,
    **{key: BOOLEAN_VALUES for key in _BOOLEAN_KEYS},
}


def _options(key: str) -> tuple[str, ...]:
    return _OPTIONS.get(key, ())


def _refuse_value(reason: str):
    raise ConfigError(reason)


def _integer(key: str, value: str, low: int, high: int | None) -> str:
    try:
        number = int(value)
    except ValueError:
        return _refuse_value(f"{key} is an integer")
    if number < low or (high is not None and number > high):
        ceiling = high if high is not None else "no maximum"
        return _refuse_value(f"{key} is between {low} and {ceiling}")
    return str(number)


def _writable(path: Path) -> bool:
    """Whether this process can replace ``path`` itself.

    The directory decides, not the file: replacing is a rename, and a file the
    agent cannot open for writing is still replaceable when it owns the
    directory. On the Anvil the loader directory is the account's and /etc is
    not, which is exactly the split this answers.
    """
    return os.access(path.parent, os.W_OK) and (not path.exists() or os.access(path, os.W_OK))


def _write_atomic(path: Path, text: str) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(text, encoding="utf-8")
    os.replace(tmp, path)
