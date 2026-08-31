"""An adapter that drives nothing, for tests and a dry run on a laptop.

The verb router is where the behaviour worth testing lives, and testing it needs
a robot that answers. This is that robot: it holds its state in memory, answers
every verb, and imports nothing a real robot needs. The suite therefore runs with
no router, no docker, and no hardware.

It is also what ``--fake`` serves, so an operator can point the desktop at a
laptop and see the surface work before a robot is on the bench.
"""

from __future__ import annotations

from fm_robot_agent.protocol import Outcome

MODES = ("idle", "teleop")


class FakeAdapter:
    """A robot that says yes, and remembers what it was told."""

    kind = "fake"

    def __init__(self) -> None:
        self.mode = MODES[0]
        self.running = False
        self.recording: str | None = None
        self._episodes: dict[str, list[dict]] = {}

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
        if config not in MODES:
            return Outcome(ok=False, message=f"unknown mode {config!r}")
        self.mode = config
        return Outcome(ok=True, message=f"mode {config}")

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
