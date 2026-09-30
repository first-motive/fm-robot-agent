"""A LeRobot SO-101 leader/follower pair, hosted by whichever machine it is plugged into.

The SO-101 has no computer of its own: each arm is a Feetech bus behind a USB
serial board. So unlike the Anvil and the Axol, the robot is not a host — it is a
pair of arms that any tailnet machine can host. The name and kind therefore come
from a robot profile on the hosting machine, not from that machine's identity
card, and the host keeps whatever identity it already has.

One process owns the arms: fm-teleop's ``fm-teleop-so101`` stack, which runs
LeRobot's own control loop and dataset writer behind a loopback HTTP API. When
this adapter hosts the robot it starts that stack as a child process on ``up``
and ends it on ``down``; the stack's ``stop`` turns the follower's torque off.

Modes are the stack's: ``idle`` (torque off) and ``teleop`` (the follower copies
the leader). Recording happens inside ``teleop``, one take at a time, into a
LeRobot v3 dataset under the profile's data root.
"""

from __future__ import annotations

import json
import math
import os
import platform
import re
import shutil
import subprocess
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path

from fm_robot_agent.axol import lerobot_v3_episodes
from fm_robot_agent.cdr import joint_state
from fm_robot_agent.config import MODE, MODE_ALIAS, Setting
from fm_robot_agent.protocol import AdapterError, Outcome

KIND = "so101"
MODES = ("idle", "teleop")
#: The stack's loopback port: clear of the Anvil agent's old 8770 and the
#: policy server's 8765, so one host can run them side by side.
STACK_PORT = 8790
#: Every stack route answers from memory; five seconds is a stack that is gone.
REQUEST_TIMEOUT_S = 5.0
#: How long `up` waits for the stack to connect both arms and answer. Adopting a
#: calibration from the motors on a new host reads twelve motors first.
STACK_START_S = 30.0
#: Enough for the Desktop's joint strip, and a tenth of the loop's bus reads.
TELEMETRY_HZ = 10.0
#: The one joint LeRobot reports as 0..100 rather than degrees.
GRIPPER = "gripper"
#: A hosted robot's name becomes a Zenoh key and a file name, so it is held to
#: the fleet's device-name shape and can carry nothing else.
ROBOT_NAME = re.compile(r"^fm-rob-[0-9]{2}$")
PROFILE_KEYS = ("kind", "leader_port", "follower_port", "leader_id", "follower_id", "root", "stack_project")


def profile_path(name: str) -> Path:
    """Where the hosting machine keeps a robot's profile.

    Beside the machine's own card on macOS, under ``/etc/fm`` on Linux — the
    same two homes the card has, because the profile is the card's counterpart
    for a robot that is not a machine.
    """
    if not ROBOT_NAME.match(name):
        raise AdapterError(f"robot name {name!r} must be fm-rob-NN, two digits (fm-rob-03)")
    if platform.system() == "Darwin":
        config_home = os.environ.get("XDG_CONFIG_HOME", "").strip()
        base = Path(config_home) if config_home else Path.home() / ".config"
        return base / "fm" / "robots" / f"{name}.json"
    return Path("/etc/fm/robots") / f"{name}.json"


@dataclass(frozen=True)
class Profile:
    """What a host needs to run one SO-101 pair it did not ship with.

    The ports may be ``serial:<USB serial number>``, which the stack resolves on
    any host — the board's serial number travels with the arm, a ``/dev`` path
    does not.
    """

    name: str
    leader_port: str
    follower_port: str
    leader_id: str
    follower_id: str
    root: str
    stack_project: str

    @classmethod
    def read(cls, name: str) -> Profile:
        path = profile_path(name)
        try:
            raw = json.loads(path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            raise AdapterError(
                f"{path} does not exist; create it once on this host with "
                f"`fm robot host {name} --leader-port serial:<n> --follower-port serial:<n> ...`"
            ) from None
        except (OSError, json.JSONDecodeError) as exc:
            raise AdapterError(f"{path} is not readable as JSON: {exc}") from exc
        if not isinstance(raw, dict) or raw.get("kind") != KIND:
            raise AdapterError(f"{path} is not an {KIND} profile")
        missing = [key for key in PROFILE_KEYS if not isinstance(raw.get(key), str) or not raw[key]]
        if missing:
            raise AdapterError(f"{path} is missing {', '.join(missing)}")
        return cls(name=name, **{key: raw[key] for key in PROFILE_KEYS if key != "kind"})

    def write(self) -> Path:
        path = profile_path(self.name)
        path.parent.mkdir(parents=True, exist_ok=True)
        body = {"kind": KIND, **{key: getattr(self, key) for key in PROFILE_KEYS if key != "kind"}}
        path.write_text(json.dumps(body, indent=2) + "\n", encoding="utf-8")
        return path

    def stack_command(self, port: int) -> list[str]:
        return [
            "uv", "run", "--project", self.stack_project, "fm-teleop-so101",
            "--leader-port", self.leader_port, "--follower-port", self.follower_port,
            "--leader-id", self.leader_id, "--follower-id", self.follower_id,
            "--owner", self.name, "--root", self.root, "--port", str(port),
        ]  # fmt: skip


class So101Adapter:
    """One SO-101 pair, reached through the stack on this host's loopback."""

    kind = KIND

    def __init__(self, profile: Profile, port: int = STACK_PORT) -> None:
        self.profile = profile
        self.base_url = f"http://127.0.0.1:{port}"
        self.port = port
        self._stack: subprocess.Popen | None = None

    # --- the verb set --------------------------------------------------------

    def status(self) -> dict:
        try:
            stack = self._call("GET", "/status")
        except AdapterError:
            stack = None
        running = stack is not None
        mode = stack["mode"] if running else None
        return {
            # Recording is a take inside teleop, not a mode of its own.
            "mode": "teleop" if mode == "recording" else mode,
            "modes": list(MODES),
            "hardware": "connected" if running else "stack down",
            "recording": bool(running and stack.get("recording")),
            "services": [{"name": "fm-teleop-so101", "state": "running" if running else "stopped"}],
            "disk": self._disk(),
            "memory": None,
            "motors": 12 if running else None,
            "take": stack.get("recording") if running else None,
            "last_episode": stack.get("last_episode") if running else None,
        }

    def up(self) -> Outcome:
        if self._reachable():
            return Outcome(ok=True, message="stack already running")
        # tradeoff: one append-only log, a few lines per start; rotate if a host
        # ever restarts the stack often enough for it to matter.
        log_path = Path(self.profile.root) / "fm-teleop-so101.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open("ab") as log:
            # The agent itself runs under `uv run`; a VIRTUAL_ENV inherited from
            # it points the stack's own `uv run` at the agent's environment.
            env = {key: value for key, value in os.environ.items() if key != "VIRTUAL_ENV"}
            self._stack = subprocess.Popen(
                self.profile.stack_command(self.port), stdout=log, stderr=subprocess.STDOUT, env=env
            )
        deadline = time.monotonic() + STACK_START_S
        while time.monotonic() < deadline:
            if self._stack.poll() is not None:
                return Outcome(ok=False, message=f"stack exited ({self._stack.returncode}); see {log_path}")
            if self._reachable():
                return Outcome(ok=True, message="both arms connected; mode idle")
            time.sleep(0.5)
        return Outcome(ok=False, message=f"stack did not answer in {STACK_START_S:.0f} s; see {log_path}")

    def down(self) -> Outcome:
        if self._stack is None or self._stack.poll() is not None:
            # Nothing of ours to stop. That is only success if no stack answers
            # at all: one started by hand still holds the arms, and saying "down"
            # would be a lie.
            return Outcome(ok=not self._reachable(), message="no stack started by this agent is running")
        self._stack.terminate()
        try:
            self._stack.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self._stack.kill()
            self._stack.wait()
        return Outcome(ok=True, message="stack stopped; follower torque off")

    def set_mode(self, config: str, args: dict[str, str] | None = None) -> Outcome:
        return self._post("/mode", {"mode": config}, f"mode {config}")

    def config_read(self) -> list[Setting]:
        mode = self.status()["mode"]
        return [Setting(key=MODE_ALIAS, value=mode, klass=MODE, options=MODES)]

    def config_write(self, key: str, value: str) -> Outcome:
        if key != MODE_ALIAS:
            return Outcome(ok=False, message=f"the SO-101 has no key {key!r}; its one setting is {MODE_ALIAS}")
        return self.set_mode(value)

    def config_rollback(self) -> Outcome:
        return Outcome(ok=False, message="no change is open")

    def session(self, action: str, dataset: str = "") -> Outcome:
        return Outcome(ok=False, message="the SO-101 records by take, with no shared session to pin")

    def record(self, dataset: str, action: str, task: str = "", note: str = "", episode: str = "") -> Outcome:
        if note or episode:
            return Outcome(ok=False, message="the SO-101 stack takes no stop note or episode check")
        if action == "start":
            return self._post("/record", {"action": "start", "dataset": dataset, "task": task},
                              f"recording into {dataset}")  # fmt: skip
        outcome = self._post("/record", {"action": "stop"}, "take saved")
        last = (outcome.detail.get("last_episode") or {}) if outcome.ok else {}
        return Outcome(ok=outcome.ok, message=outcome.message, detail={"episode": str(last.get("episode", ""))})

    def stop(self) -> Outcome:
        """End any take (saved) and turn the follower's torque off.

        Not an emergency stop: the SO-101 has none, and the arm drops under its
        own weight when torque goes. The physical stop is the follower's supply.
        """
        return self._post("/stop", {}, "follower torque off")

    def episodes(self, dataset: str) -> list[dict]:
        root = Path(self.profile.root) / dataset
        if not (root / "meta" / "info.json").is_file():
            return []
        return lerobot_v3_episodes(root)

    # --- telemetry -----------------------------------------------------------

    def telemetry(self):
        """Yield the follower's joints as CDR ``sensor_msgs/JointState``.

        LeRobot reports joints in degrees and the gripper as 0..100; the fleet
        speaks radians, so the joints are converted and the gripper becomes
        0..1. Ends when the stack stops answering; the caller reopens it.
        """
        while True:
            joints = self._call("GET", "/status").get("joints") or {}
            names = sorted(joints)
            positions = [
                joints[n] / 100.0 if n == GRIPPER else math.radians(joints[n]) for n in names
            ]
            yield "joint_states", joint_state(
                stamp_s=time.time(), names=names, positions=positions,
                velocities=[0.0] * len(names), efforts=[0.0] * len(names),
            )  # fmt: skip
            time.sleep(1.0 / TELEMETRY_HZ)

    # --- the stack -----------------------------------------------------------

    def _disk(self) -> dict | None:
        try:
            usage = shutil.disk_usage(self.profile.root)
        except OSError:
            return None
        return {"total_kb": usage.total // 1024, "available_kb": usage.free // 1024}

    def _reachable(self) -> bool:
        try:
            self._call("GET", "/status")
        except AdapterError:
            return False
        return True

    def _post(self, path: str, body: dict, done: str) -> Outcome:
        try:
            answer = self._call("POST", path, body)
        except _Refused as refusal:
            return Outcome(ok=False, message=str(refusal))
        return Outcome(ok=True, message=done, detail=answer)

    def _call(self, method: str, path: str, body: dict | None = None) -> dict:
        data = json.dumps(body).encode() if body is not None else None
        request = urllib.request.Request(
            self.base_url + path, data=data, method=method, headers={"Content-Type": "application/json"}
        )
        try:
            with urllib.request.urlopen(request, timeout=REQUEST_TIMEOUT_S) as response:
                return json.loads(response.read() or b"{}")
        except urllib.error.HTTPError as exc:
            try:
                reason = json.loads(exc.read() or b"{}").get("error") or exc.reason
            except json.JSONDecodeError:
                reason = exc.reason
            raise _Refused(reason) from exc
        except (urllib.error.URLError, OSError, json.JSONDecodeError) as exc:
            raise AdapterError(f"the SO-101 stack is not answering on {self.base_url}: {exc}") from exc


class _Refused(AdapterError):
    """The stack answered and said no."""
