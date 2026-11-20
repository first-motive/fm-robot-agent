"""The fm-axol units, against a stub of ``systemctl`` at the subprocess boundary."""

from __future__ import annotations

import subprocess

import pytest

from fm_robot_agent.axol_ros import (
    CAMERA_UNIT,
    CORE_UNIT,
    ROS_UNIT,
    SUDOERS_RULE,
    RosStack,
    holds_buses,
)

#: The ways sudo words a refusal, verbatim from sudo 1.9 on Ubuntu 22.04.
SUDO_REFUSALS = [
    "sudo: a password is required\n",
    f"Sorry, user axol is not allowed to execute '/usr/bin/systemctl start {CORE_UNIT}' as root on fm-rob-02.\n",
    "Sorry, user axol may not run sudo on fm-rob-02.\n",
]


def test_start_brings_up_the_core_before_the_container(systemctl):
    assert RosStack().start() is None
    assert systemctl.changes() == [("start", CORE_UNIT), ("start", ROS_UNIT)]
    assert RosStack().active() is True


def test_stop_takes_down_the_container_before_the_core(systemctl):
    stack = RosStack()
    stack.start()
    systemctl.calls.clear()
    assert stack.stop() is None
    assert systemctl.changes() == [("stop", ROS_UNIT), ("stop", CORE_UNIT)]
    assert stack.active() is False


def test_starts_and_stops_run_one_unit_each_under_non_interactive_sudo(systemctl, monkeypatch):
    """The sudoers rule matches one exact command line per unit; nothing else may reach it."""
    seen = []

    def record(argv, **kwargs):
        seen.append(list(argv))
        return systemctl.run(argv, **kwargs)

    monkeypatch.setattr(subprocess, "run", record)
    stack = RosStack()
    stack.start()
    stack.stop()
    privileged = [argv for argv in seen if argv[0] == "/usr/bin/sudo"]
    assert privileged == [
        ["/usr/bin/sudo", "-n", "/usr/bin/systemctl", "start", CORE_UNIT],
        ["/usr/bin/sudo", "-n", "/usr/bin/systemctl", "start", ROS_UNIT],
        ["/usr/bin/sudo", "-n", "/usr/bin/systemctl", "stop", ROS_UNIT],
        ["/usr/bin/sudo", "-n", "/usr/bin/systemctl", "stop", CORE_UNIT],
    ]
    assert all(argv[:2] == ["systemctl", "is-active"] for argv in seen if argv[0] != "/usr/bin/sudo")


@pytest.mark.parametrize("refusal", SUDO_REFUSALS, ids=["password", "not-allowed", "no-rule"])
def test_a_sudo_refusal_is_a_failed_start_that_names_the_rule_in_the_journal(systemctl, refusal, capsys):
    """The rule's path and sudo's own words stay on the host; the fabric gets the short reason."""
    systemctl.sudo_refusal = refusal
    failure = RosStack().start()
    assert failure.startswith(f"sudo refused systemctl start {CORE_UNIT}")
    assert SUDOERS_RULE not in failure
    journal = capsys.readouterr().err
    assert SUDOERS_RULE in journal and refusal.strip() in journal
    assert ("start", ROS_UNIT) not in systemctl.changes()


def test_a_rollback_that_cannot_stop_the_core_says_so(systemctl):
    """A refused stop leaves the core on the buses; the reason must not claim otherwise."""
    systemctl.fail_start.add(ROS_UNIT)
    systemctl.fail_stop.add(CORE_UNIT)
    failure = RosStack().start()
    assert "rollback did not stop the units" in failure and CORE_UNIT in failure
    assert "both units stopped" not in failure
    assert RosStack().active() is True


def test_a_container_that_fails_to_start_rolls_the_core_back(systemctl):
    systemctl.fail_start.add(ROS_UNIT)
    failure = RosStack().start()
    assert failure is not None and ROS_UNIT in failure and "failed" in failure
    assert systemctl.changes()[-2:] == [("stop", ROS_UNIT), ("stop", CORE_UNIT)]
    assert systemctl.states[CORE_UNIT] == "inactive"


def test_a_core_that_dies_on_start_is_a_failed_start(systemctl):
    """`systemctl start` returns once the process forks; the state says if it stayed."""
    systemctl.die_on_start.add(CORE_UNIT)
    failure = RosStack().start()
    assert failure == f"{CORE_UNIT} is failed after start; both units stopped"
    assert ("start", ROS_UNIT) not in systemctl.changes(), "the container never starts without its core"


def test_stop_still_stops_the_core_when_the_container_will_not_stop(systemctl):
    stack = RosStack()
    stack.start()
    systemctl.fail_stop.add(ROS_UNIT)
    failure = stack.stop()
    assert failure is not None and ROS_UNIT in failure
    assert systemctl.states[CORE_UNIT] == "inactive"


def test_a_host_without_systemctl_reports_unknown_and_holds_no_bus(monkeypatch):
    def missing(*_args, **_kwargs):
        raise FileNotFoundError("systemctl")

    monkeypatch.setattr(subprocess, "run", missing)
    stack = RosStack()
    assert stack.states() == {CORE_UNIT: "absent", ROS_UNIT: "absent", CAMERA_UNIT: "absent"}
    assert stack.active() is False
    assert "could not run" in stack.start()


def test_the_cameras_start_on_their_own_once_the_stack_is_up(systemctl):
    stack = RosStack()
    stack.start()
    assert ("start", CAMERA_UNIT) not in systemctl.changes()
    assert stack.start_cameras() is None
    assert systemctl.changes()[-1] == ("start", CAMERA_UNIT)
    assert stack.states()[CAMERA_UNIT] == "active"


def test_a_camera_start_runs_under_the_same_sudo_rule(systemctl, monkeypatch):
    seen = []

    def record(argv, **kwargs):
        seen.append(list(argv))
        return systemctl.run(argv, **kwargs)

    monkeypatch.setattr(subprocess, "run", record)
    RosStack().start_cameras()
    assert ["/usr/bin/sudo", "-n", "/usr/bin/systemctl", "start", CAMERA_UNIT] in seen


def test_a_camera_that_fails_to_start_leaves_the_stack_running(systemctl):
    """Motion needs no camera, so a camera failure rolls nothing back."""
    stack = RosStack()
    stack.start()
    systemctl.fail_start.add(CAMERA_UNIT)
    failure = stack.start_cameras()
    assert failure is not None and CAMERA_UNIT in failure
    assert ("stop", CORE_UNIT) not in systemctl.changes()
    assert stack.active() is True


def test_stop_releases_the_cameras_before_the_container_and_core(systemctl):
    stack = RosStack()
    stack.start()
    stack.start_cameras()
    systemctl.calls.clear()
    assert stack.stop() is None
    assert systemctl.changes() == [("stop", CAMERA_UNIT), ("stop", ROS_UNIT), ("stop", CORE_UNIT)]
    assert stack.idle() is True


def test_stop_leaves_a_camera_unit_that_is_not_running_alone(systemctl):
    """An fm-axol install without the camera unit has no sudoers line for it either."""
    stack = RosStack()
    stack.start()
    systemctl.calls.clear()
    assert stack.stop() is None
    assert ("stop", CAMERA_UNIT) not in systemctl.changes()


def test_stop_still_stops_the_stack_when_the_cameras_will_not_stop(systemctl):
    stack = RosStack()
    stack.start()
    stack.start_cameras()
    systemctl.fail_stop.add(CAMERA_UNIT)
    failure = stack.stop()
    assert failure is not None and CAMERA_UNIT in failure
    assert systemctl.states[CORE_UNIT] == systemctl.states[ROS_UNIT] == "inactive"


def test_a_running_camera_unit_holds_no_bus_but_is_not_idle(systemctl):
    systemctl.states[CAMERA_UNIT] = "active"
    stack = RosStack()
    assert stack.active() is False
    assert stack.idle() is False


def test_a_unit_on_its_way_up_or_down_still_holds_the_buses():
    assert holds_buses({CORE_UNIT: "activating", ROS_UNIT: "inactive"})
    assert holds_buses({CORE_UNIT: "deactivating", ROS_UNIT: "inactive"})
    assert holds_buses({CORE_UNIT: "maintenance", ROS_UNIT: "inactive"})
    assert not holds_buses({CORE_UNIT: "failed", ROS_UNIT: "inactive"})


def test_a_state_read_that_answers_nothing_holds_the_buses(monkeypatch):
    """A D-Bus or polkit error exits non-zero with only stderr; that is not idle."""

    def broken(argv, **_kwargs):
        return subprocess.CompletedProcess(argv, 1, "", "Failed to connect to bus\n")

    monkeypatch.setattr(subprocess, "run", broken)
    assert RosStack().active() is True


def test_a_hung_state_read_holds_the_buses(monkeypatch):
    """Exclusion fails closed: a unit nobody could read may still be running."""

    def hang(argv, **kwargs):
        raise subprocess.TimeoutExpired(argv, kwargs.get("timeout"))

    monkeypatch.setattr(subprocess, "run", hang)
    assert RosStack().active() is True
