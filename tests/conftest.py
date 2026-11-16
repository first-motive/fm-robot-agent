"""Fixtures shared by more than one test module."""

from __future__ import annotations

import subprocess

import pytest


class SystemctlStub:
    """``systemctl`` for the fm-axol units, holding each unit's state in memory.

    Patched in at ``subprocess.run``, so the code under test builds its own argv
    and reads its own return codes, exactly as it would on the robot.
    """

    def __init__(self) -> None:
        self.calls: list[tuple[str, str]] = []
        self.states: dict[str, str] = {}
        #: Units whose start exits non-zero, as a unit with a failing ExecStart does.
        self.fail_start: set[str] = set()
        #: Units that start cleanly but are not active afterwards, as one that dies at once does.
        self.die_on_start: set[str] = set()
        #: Units whose stop exits non-zero.
        self.fail_stop: set[str] = set()

    def run(self, argv, **_kwargs) -> subprocess.CompletedProcess:
        assert argv[0] == "systemctl" and len(argv) == 3, argv
        action, unit = argv[1], argv[2]
        self.calls.append((action, unit))
        if action == "is-active":
            state = self.states.get(unit, "inactive")
            return subprocess.CompletedProcess(argv, 0 if state == "active" else 3, f"{state}\n", "")
        if action == "start":
            if unit in self.fail_start:
                return subprocess.CompletedProcess(argv, 1, "", f"Job for {unit} failed.\n")
            self.states[unit] = "failed" if unit in self.die_on_start else "active"
            return subprocess.CompletedProcess(argv, 0, "", "")
        if action == "stop":
            if unit in self.fail_stop:
                return subprocess.CompletedProcess(argv, 1, "", f"Failed to stop {unit}.\n")
            self.states[unit] = "inactive"
            return subprocess.CompletedProcess(argv, 0, "", "")
        raise AssertionError(f"unexpected systemctl action {action!r}")

    def changes(self) -> list[tuple[str, str]]:
        """The starts and stops, in order, without the state reads between them."""
        return [call for call in self.calls if call[0] != "is-active"]


@pytest.fixture
def systemctl(monkeypatch) -> SystemctlStub:
    stub = SystemctlStub()
    monkeypatch.setattr(subprocess, "run", stub.run)
    return stub
