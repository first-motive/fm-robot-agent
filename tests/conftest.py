"""Fixtures shared by more than one test module."""

from __future__ import annotations

import subprocess

import pytest

from fm_robot_agent import units


class SystemctlStub:
    """``systemctl`` for the fm-axol units, holding each unit's state in memory.

    Patched in at ``subprocess.run``, so the code under test builds its own argv
    and reads its own return codes, exactly as it would on the robot. Starts and
    stops must arrive as the exact argv fm-axol's sudoers rule matches; anything
    else is a test failure, as it would be a sudo refusal on the robot.
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
        #: sudo's stderr for every privileged call, or ``None`` to let it through.
        self.sudo_refusal: str | None = None
        #: Almond's gripper calibration as root's file holds it; ``None`` when absent.
        self.gripper_calibration: str | None = None
        self.gripper_reads = 0

    def run(self, argv, **_kwargs) -> subprocess.CompletedProcess:
        from fm_robot_agent.axol import READ_GRIPPER_CALIBRATION

        if tuple(argv) == READ_GRIPPER_CALIBRATION:
            # Only this exact argv: fm-axol's sudoers line matches it and nothing else.
            self.gripper_reads += 1
            if self.gripper_calibration is None:
                return subprocess.CompletedProcess(argv, 1, "", "cat: No such file or directory\n")
            return subprocess.CompletedProcess(argv, 0, self.gripper_calibration, "")
        if argv[:2] == ["systemctl", "is-active"]:
            assert len(argv) == 3, argv
            action, unit = "is-active", argv[2]
        else:
            assert argv[:3] == ["/usr/bin/sudo", "-n", "/usr/bin/systemctl"] and len(argv) == 5, argv
            action, unit = argv[3], argv[4]
            assert action in ("start", "stop"), argv
        self.calls.append((action, unit))
        if action != "is-active" and self.sudo_refusal is not None:
            return subprocess.CompletedProcess(argv, 1, "", self.sudo_refusal)
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


@pytest.fixture(autouse=True)
def no_settle_window(monkeypatch):
    """A start reads its unit once. Tests that need the window set it themselves."""
    monkeypatch.setattr(units, "SETTLE_S", 0)
