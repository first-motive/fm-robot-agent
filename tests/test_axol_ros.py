"""The fm-axol units, against a stub of ``systemctl`` at the subprocess boundary."""

from __future__ import annotations

import subprocess

from fm_robot_agent.axol_ros import CORE_UNIT, ROS_UNIT, RosStack, holds_buses


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
    assert failure == f"{CORE_UNIT} is failed after start"
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
    assert stack.states() == {CORE_UNIT: "absent", ROS_UNIT: "absent"}
    assert stack.active() is False
    assert "could not run" in stack.start()


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
