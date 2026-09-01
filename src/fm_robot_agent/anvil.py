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

import os
import re
import shlex
import shutil
import subprocess
from pathlib import Path

from fm_robot_agent.protocol import AdapterError, Outcome
from fm_robot_agent.trpc import TrpcClient

KIND = "anvil-openarm-v2"

MODE_KEY = "ARMS_CONTROL_CONFIG_FILE"
COMPOSE_LOG = Path("/tmp/fm-robot-agent-compose.log")

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

    def __init__(self, loader_dir: Path | None = None, webapp_url: str | None = None) -> None:
        self.loader_dir = loader_dir or Path(
            os.environ.get("FM_ANVIL_LOADER_DIR", str(Path.home() / "anvil-loader"))
        )
        self.config_dir = self.loader_dir / "config"
        self.env_config = self.loader_dir / ".env.config"
        self.recordings_dir = self.loader_dir / "data" / "recordings"
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
        """Rewrite the mode line, then recreate the stack so every container sees it."""
        if config not in self.list_configs():
            return Outcome(ok=False, message=f"unknown config {config!r}")
        self.write_mode(config)
        applied = self._compose_detached("recreate")
        return Outcome(
            ok=applied.ok,
            message=f"mode {config}; {applied.message}",
            detail={"mode": config},
        )

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

    def write_mode(self, name: str) -> None:
        """Rewrite the mode line idempotently and atomically.

        Drop any existing assignment, append the new one, replace the file — so a
        repeated call leaves one line and every other key survives untouched.
        """
        try:
            lines = [
                line
                for line in self.env_config.read_text(encoding="utf-8").splitlines()
                if not line.startswith(MODE_KEY + "=")
            ]
        except OSError:
            lines = []
        lines.append(f"{MODE_KEY}={name}")
        tmp = self.env_config.with_suffix(self.env_config.suffix + ".tmp")
        tmp.write_text("\n".join(lines) + "\n", encoding="utf-8")
        os.replace(tmp, self.env_config)

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

    def _service_call(self, service: str, service_type: str, request: str) -> str:
        """Call a ROS service from inside the ros2 container, and return its output.

        ``anvil_msgs`` exists only in that container, so the call cannot be made
        from the host. Argument list, never a shell — the arguments are fixed
        here and no caller-supplied value reaches them.
        """
        # One `bash -c` because the sourcing and the call have to share a shell.
        # Every value in it is a constant from this module — no caller-supplied
        # string reaches the command line.
        # The two env-supplied values are quoted like the arguments are. They come
        # from this host's own config rather than the fabric, but a path and an
        # RMW name are still values someone typed, and quoting costs nothing.
        inner = (
            f"source {ROS_SETUP} && source {shlex.quote(WORKSPACE_SETUP)} && "
            f"RMW_IMPLEMENTATION={shlex.quote(RMW)} "
            f"ros2 service call {shlex.quote(service)} {shlex.quote(service_type)} {shlex.quote(request)}"
        )
        try:
            result = subprocess.run(
                ["docker", "compose", "exec", "-T", "ros2", "bash", "-c", inner],
                cwd=self.loader_dir,
                capture_output=True,
                text=True,
                timeout=SERVICE_TIMEOUT_S,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise AdapterError(f"{service} did not answer: {exc}") from exc
        if result.returncode != 0:
            raise AdapterError(f"{service} failed: {result.stderr.strip() or 'no output'}")
        return result.stdout

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
