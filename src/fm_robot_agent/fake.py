"""An adapter that drives nothing, for tests and a dry run on a laptop.

The verb router is where the behaviour worth testing lives, and testing it needs
a robot that answers. This is that robot: it holds its state in memory, answers
every verb, and imports nothing a real robot needs. The suite therefore runs with
no router, no docker, and no hardware.

It is also what ``--fake`` serves, so an operator can point the desktop at a
laptop and see the surface work before a robot is on the bench.
"""

from __future__ import annotations

from fm_robot_agent.config import (
    MODE,
    MODE_ALIAS,
    MOTION,
    SEVERING,
    TUNING,
    UNKNOWN,
    Setting,
)
from fm_robot_agent.protocol import Outcome

MODES = ("idle", "teleop")

#: One key of every class, because what the suite needs to exercise is the guard
#: each class carries rather than any robot's real key set. The names are the
#: Anvil's, so a fixture reads like the robot a reader has in mind.
FAKE_KEYS = {
    "ARMS_CONTROL_CONFIG_FILE": MODE,
    "CYCLONEDDS_IFACE": SEVERING,
    "TELEOP_POSITION_SCALE": MOTION,
    "CYCLONEDDS_VERBOSITY": TUNING,
    "ANVIL_SOMETHING_NEW": UNKNOWN,
}


class FakeAdapter:
    """A robot that says yes, and remembers what it was told."""

    kind = "fake"

    def __init__(self) -> None:
        self.mode = MODES[0]
        self.running = False
        self.recording: str | None = None
        self._episodes: dict[str, list[dict]] = {}
        self.config = {
            "ARMS_CONTROL_CONFIG_FILE": MODES[0],
            "CYCLONEDDS_IFACE": "eth0",
            "TELEOP_POSITION_SCALE": "1.0",
            "CYCLONEDDS_VERBOSITY": "warning",
            "ANVIL_SOMETHING_NEW": "whatever the vendor added",
        }
        #: What a severing write would restore. The fake never severs anything,
        #: so it only has to remember that a window is open.
        self.pending: tuple[str, str] | None = None

    def status(self) -> dict:
        return {
            "mode": self.mode,
            "hardware": "running" if self.running else "down",
            "recording": self.recording,
            "services": [{"name": "fake", "state": "running" if self.running else "exited"}],
            "disk": {"total_kb": 0, "available_kb": 0},
        }

    def up(self) -> Outcome:
        self.running = True
        return Outcome(ok=True, message="fake robot up")

    def down(self) -> Outcome:
        self.running = False
        return Outcome(ok=True, message="fake robot down")

    def set_mode(self, config: str) -> Outcome:
        return self.config_write(MODE_ALIAS, config)

    def config_read(self) -> list[Setting]:
        return [
            Setting(key=key, value=self.config[key], klass=klass)
            for key, klass in sorted(FAKE_KEYS.items())
        ]

    def config_write(self, key: str, value: str) -> Outcome:
        if key == MODE_ALIAS:
            key = "ARMS_CONTROL_CONFIG_FILE"
        klass = FAKE_KEYS.get(key, UNKNOWN)
        if klass == UNKNOWN or key not in self.config:
            return Outcome(ok=False, message=f"{key} is unclassified and cannot be written")
        if klass == MOTION and self.running:
            return Outcome(ok=False, message=f"{key} is a motion key and this robot is busy")
        if klass == MODE and value not in MODES:
            return Outcome(ok=False, message=f"unknown mode {value!r}")
        previous = self.config[key]
        self.config[key] = value
        if klass == MODE:
            self.mode = value
        if klass == SEVERING:
            self.pending = (key, previous)
        return Outcome(ok=True, message=f"{key}={value}", detail={"class": klass})

    def config_rollback(self) -> Outcome:
        if self.pending is None:
            return Outcome(ok=False, message="no change is open")
        key, previous = self.pending
        self.config[key] = previous
        self.pending = None
        return Outcome(ok=True, message=f"{key} restored", detail={"key": key})

    def record(self, dataset: str, action: str) -> Outcome:
        if action == "start":
            if self.recording is not None:
                return Outcome(ok=False, message=f"already recording into {self.recording}")
            episodes = self._episodes.setdefault(dataset, [])
            slug = f"{len(episodes):04d}"
            episodes.append({"slug": slug, "size_kb": 0})
            self.recording = dataset
            return Outcome(ok=True, message="recording", detail={"episode": slug})
        if self.recording is None:
            return Outcome(ok=False, message="not recording")
        self.recording = None
        return Outcome(ok=True, message="stopped")

    def stop(self) -> Outcome:
        self.recording = None
        self.mode = MODES[0]
        return Outcome(ok=True, message="stopped", detail={"state": "idle"})

    def episodes(self, dataset: str) -> list[dict]:
        return list(self._episodes.get(dataset, []))
