"""The Almond Axol, driven through its own web stack.

The Axol has no ROS graph. One process — Almond's FastAPI server on
``https://localhost:8001`` — owns the CAN bus, the cameras, and every operation
the robot runs, and it is installed from Almond's own release. It serves TLS with
a certificate it signed itself, which is checked for every host except loopback:
the agent runs on the same machine, so that connection never leaves it, and no
authority issues a certificate for ``localhost``. This adapter is a client of
that server and nothing more: it imports no Almond SDK, touches no CAN interface,
and starts no process the server does not already manage. A stock Axol install
keeps working with the agent installed beside it.

Two things need saying about the mapping:

*Modes are operations.* The Axol runs one operation at a time — teleoperation,
gravity compensation, a trained policy — so switching mode is stopping whatever
runs and starting the next. The names the fleet uses (``teleop``,
``gravity-comp``, ``policy``) map onto the server's operation ids.

*Datasets are LeRobot repo ids.* A repo id carries a ``/``, which is exactly the
character a dataset name may not contain. The owner half is configuration, the
caller supplies only the name half, and the two are joined here.
"""

from __future__ import annotations

import ipaddress
import json
import os
import re
import ssl
import time
import urllib.error
import urllib.request
from pathlib import Path
from urllib.parse import urlsplit

from fm_robot_agent.cdr import joint_state
from fm_robot_agent.protocol import AdapterError, Outcome

KIND = "axol"

# HTTPS, on the robot's own loopback. Almond's server presents a self-signed
# certificate — its web UI has an "accept this certificate" page for exactly that
# reason — and nothing issues it a real one, because it is never meant to be
# reached from off the host.
DEFAULT_BASE_URL = "https://localhost:8001"
DEFAULT_DATASET_OWNER = "axol"

#: What the fleet calls a mode, and the operation id the Axol server runs for it.
MODES = {
    "teleop": "teleop",
    "gravity-comp": "gravity-comp",
    "policy": "run-policy",
}

RECORD_OPERATION = "collect-data"

#: The owner half of a repo id becomes a directory component, exactly as the name
#: half does, so it is held to the same shape. Configuration is trusted less than
#: it is convenient to assume: an owner read from the environment is still a value
#: someone typed.
OWNER_PATTERN = re.compile(r"^[a-z0-9]+(?:-[a-z0-9]+)*$")

#: Telemetry ceiling. The Axol samples its motors at 10 Hz, so this is a cap that
#: no upsampling ever fills — the agent forwards frames as they arrive and drops
#: any that would exceed it, rather than inventing samples between them.
TELEMETRY_MAX_HZ = 20.0

REQUEST_TIMEOUT_S = 10.0

#: Hosts whose certificate is not checked. The agent and the Axol server run on
#: the same machine, so the connection never leaves it: there is no network for a
#: man in the middle to sit in, and no authority that would issue a certificate
#: for `localhost` anyway. Anything else is verified normally — a base URL
#: pointing off-host is a different situation and gets the usual rules.
LOOPBACK_HOSTS = ("localhost",)


def _loopback(url: str) -> bool:
    """Whether a URL points at this machine.

    An address is asked whether it is loopback rather than compared to a list of
    spellings: `::1`, `0:0:0:0:0:0:0:1` and `127.0.0.2` are all loopback and only
    the first would survive a string match, which would then demand a certificate
    no one can issue and fail the connection.
    """
    host = urlsplit(url).hostname or ""
    if host in LOOPBACK_HOSTS:
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def _ssl_context(url: str) -> ssl.SSLContext | None:
    """The TLS context for a URL, or ``None`` when it is not HTTPS at all.

    Verification is dropped only for loopback, and only because the certificate
    there is self-signed by design. See :data:`LOOPBACK_HOSTS`.
    """
    if not url.startswith(("https://", "wss://")):
        return None
    context = ssl.create_default_context()
    if _loopback(url):
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
    return context


class AxolAdapter:
    """One Axol, reached through the Almond server on this host."""

    kind = KIND

    def __init__(self, base_url: str | None = None, dataset_owner: str | None = None) -> None:
        self.base_url = (base_url or os.environ.get("FM_AXOL_URL", DEFAULT_BASE_URL)).rstrip("/")
        owner = dataset_owner or os.environ.get("FM_AXOL_DATASET_OWNER", DEFAULT_DATASET_OWNER)
        if not OWNER_PATTERN.match(owner):
            raise AdapterError(f"dataset owner {owner!r} is not a plain name")
        self.dataset_owner = owner

    # --- the verb set --------------------------------------------------------

    def status(self) -> dict:
        robot = self._get("/api/robot/status")
        operation = self._get("/api/op/status")
        session = operation.get("session") or {}
        running = bool(operation.get("running"))
        return {
            "mode": self._fleet_mode(session.get("command") or session.get("op")),
            "modes": sorted(MODES),
            "hardware": robot.get("state", "unknown"),
            "recording": running and session.get("op") == RECORD_OPERATION,
            "services": [{"name": "almond-axol", "state": robot.get("state", "unknown")}],
            "disk": None,
            "motors": robot.get("motors"),
        }

    def up(self) -> Outcome:
        """Connect the robot link onto the CAN interfaces the server already holds.

        An empty body on purpose: choosing interfaces is a bench decision made in
        the Axol's own UI, and the fleet has no business re-pointing a CAN bus.
        """
        answered = self._post("/api/robot/connect", {})
        return Outcome(ok=True, message="link connected", detail={"state": answered.get("state", "")})

    def down(self) -> Outcome:
        answered = self._post("/api/robot/disconnect", {})
        return Outcome(ok=True, message="link disconnected", detail={"state": answered.get("state", "")})

    def set_mode(self, config: str) -> Outcome:
        """Stop whatever operation runs and start the one this mode names."""
        operation = MODES.get(config)
        if operation is None:
            return Outcome(ok=False, message=f"unknown mode {config!r}; expected one of {sorted(MODES)}")
        self._stop_operation()
        started = self._post("/api/op/start", {"op": operation, "args": {}, "cameras": []})
        return Outcome(
            ok=True,
            message=f"mode {config}",
            detail={"mode": config, "session": str(started.get("id", ""))},
        )

    def record(self, dataset: str, action: str) -> Outcome:
        if action == "start":
            started = self._post(
                "/api/op/start",
                {"op": RECORD_OPERATION, "args": {"repo_id": self._repo_id(dataset)}, "cameras": []},
            )
            return Outcome(
                ok=True,
                message=f"recording into {dataset}",
                detail={"episode": str(started.get("id", ""))},
            )
        stopped = self._stop_operation()
        return Outcome(ok=True, message="recording stopped", detail={"episode": str(stopped.get("id", ""))})

    def stop(self) -> Outcome:
        """Stop the running operation and disconnect the motors.

        The Axol's own halt, and the strongest one it has: a disconnected link
        holds no motor. Not an emergency stop — no e-stop exists on this robot.
        """
        self._stop_operation()
        answered = self._post("/api/robot/disconnect", {})
        return Outcome(ok=True, message="motors disconnected", detail={"state": answered.get("state", "")})

    def episodes(self, dataset: str) -> list[dict]:
        """Episodes in a LeRobot dataset on this host, from its own index."""
        root = self._dataset_root(dataset)
        index = root / "meta" / "episodes.jsonl"
        try:
            lines = index.read_text(encoding="utf-8").splitlines()
        except OSError:
            return []
        found = []
        for line in lines:
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(record, dict):
                continue
            slug = f"{record.get('episode_index', 0):06d}"
            found.append(
                {
                    "slug": slug,
                    "size_kb": _episode_size_kb(root, record.get("episode_index", 0)),
                    "length": record.get("length"),
                    "tasks": record.get("tasks"),
                }
            )
        return found

    # --- telemetry -----------------------------------------------------------

    def telemetry(self):
        """Yield ``(topic, cdr_bytes)`` for every motor frame the Axol publishes.

        The Axol's telemetry socket speaks its own JSON; the fleet speaks CDR
        ``sensor_msgs/JointState``, the same as a rig behind the ROS bridge. This
        is where the one becomes the other, so the desktop decodes both robots
        with one decoder.

        A dropped socket ends the generator; the caller decides whether to
        reconnect.
        """
        try:
            from websockets.sync.client import connect
        except ImportError as exc:  # pragma: no cover - a packaging fault, not a path
            raise AdapterError(f"websockets is not installed: {exc}") from exc

        url = self.base_url.replace("https://", "wss://").replace("http://", "ws://")
        minimum_gap_s = 1.0 / TELEMETRY_MAX_HZ
        last_sent = 0.0
        socket_url = f"{url}/api/telemetry/ws"
        with connect(
            socket_url, open_timeout=REQUEST_TIMEOUT_S, ssl=_ssl_context(socket_url)
        ) as socket:
            for message in socket:
                try:
                    frame = json.loads(message)
                except (json.JSONDecodeError, TypeError):
                    continue
                if frame.get("type") != "frame":
                    continue
                now = time.time()
                if now - last_sent < minimum_gap_s:
                    continue
                last_sent = now
                yield "joint_states", _frame_to_joint_state(frame)

    # --- datasets ------------------------------------------------------------

    def _repo_id(self, dataset: str) -> str:
        """The LeRobot repo id for a dataset name.

        The owner half is configuration and the name half is the caller's, so a
        caller can never reach a dataset outside the owner it is scoped to.
        """
        return f"{self.dataset_owner}/{dataset}"

    def _dataset_root(self, dataset: str) -> Path:
        home = os.environ.get("HF_LEROBOT_HOME")
        base = Path(home) if home else Path.home() / ".cache" / "huggingface" / "lerobot"
        return base / self.dataset_owner / dataset

    # --- the Axol server -----------------------------------------------------

    def _stop_operation(self) -> dict:
        """Stop the running operation, tolerating there being none."""
        try:
            return self._post("/api/op/stop", {})
        except AdapterError:
            return {}

    def _get(self, path: str) -> dict:
        return self._request(path, None)

    def _post(self, path: str, payload: dict) -> dict:
        return self._request(path, json.dumps(payload).encode())

    def _request(self, path: str, body: bytes | None) -> dict:
        request = urllib.request.Request(
            f"{self.base_url}{path}",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST" if body is not None else "GET",
        )
        try:
            with urllib.request.urlopen(
                request, timeout=REQUEST_TIMEOUT_S, context=_ssl_context(self.base_url)
            ) as response:
                decoded = json.loads(response.read() or b"{}")
        except urllib.error.HTTPError as exc:
            raise AdapterError(f"{path} refused: {exc.code} {exc.reason}") from exc
        except (urllib.error.URLError, OSError, json.JSONDecodeError) as exc:
            raise AdapterError(f"the Axol server at {self.base_url} did not answer: {exc}") from exc
        return decoded if isinstance(decoded, dict) else {}

    @staticmethod
    def _fleet_mode(operation_id: str | None) -> str | None:
        """The fleet's name for a running operation, or ``None`` when it has none."""
        for mode, operation in MODES.items():
            if operation == operation_id:
                return mode
        return None


def _frame_to_joint_state(frame: dict) -> bytes:
    """One Axol telemetry frame as CDR ``sensor_msgs/JointState``.

    The frame holds ``{"left:SHOULDER_1": [position, velocity, torque]}``. Motor
    keys are sorted so a subscriber sees a stable joint order across frames — the
    dict's own order is whatever the sampler produced.
    """
    motors = frame.get("m") or {}
    names = sorted(motors)
    published: list[str] = []
    positions, velocities, efforts = [], [], []
    for name in names:
        sample = motors.get(name)
        # A frame that is not three numbers is a robot misreporting itself, not a
        # joint to publish. Dropping it keeps one bad motor from costing the frame.
        if not isinstance(sample, (list, tuple)):
            continue
        try:
            values = [float(value) for value in sample[:3]]
        except (TypeError, ValueError):
            continue
        values += [0.0] * (3 - len(values))
        published.append(name)
        positions.append(values[0])
        velocities.append(values[1])
        efforts.append(values[2])
    return joint_state(
        stamp_s=float(frame.get("t") or time.time()),
        names=published,
        positions=positions,
        velocities=velocities,
        efforts=efforts,
    )


def _episode_size_kb(root: Path, episode_index: int) -> int:
    """The parquet and video bytes one episode occupies, as far as they are found."""
    total = 0
    for pattern in (f"data/**/episode_{episode_index:06d}.parquet", f"videos/**/episode_{episode_index:06d}.mp4"):
        for path in root.glob(pattern):
            try:
                total += path.stat().st_size
            except OSError:
                continue
    return total // 1024
