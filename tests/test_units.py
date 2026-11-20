"""The ``unit`` verb and the host units behind it, against a stub of ``systemctl``."""

from __future__ import annotations

import json
import subprocess

import pytest

from fm_robot_agent import units
from fm_robot_agent.axol_ros import CORE_UNIT, RosStack
from fm_robot_agent.fake import FakeAdapter
from fm_robot_agent.protocol import Outcome
from fm_robot_agent.verbs import KEY_PREFIX, answer, verb_of

from .conftest import SystemctlStub

NS = "fm_rob_02"
SUDO = ["/usr/bin/sudo", "-n"]


class HostStub(SystemctlStub):
    """The host's systemd: what is installed, and every command this verb may send."""

    def __init__(self) -> None:
        super().__init__()
        #: Installed units and their UnitFileState. A unit absent here is not-found.
        self.installed: dict[str, str] = {}
        self.argv: list[list[str]] = []

    def run(self, argv, **kwargs) -> subprocess.CompletedProcess:
        argv = list(argv)
        self.argv.append(argv)
        if argv[:2] == ["systemctl", "show"]:
            return subprocess.CompletedProcess(argv, 0, "\n".join(self._show(u) for u in argv[3:]), "")
        if argv[:2] in (["systemctl", "list-units"], ["systemctl", "list-unit-files"]):
            listed = [u for u in self.installed if u.startswith("fm-")]
            if argv[1] == "list-units":
                listed = [u for u in listed if self.states.get(u, "inactive") != "inactive" or u.endswith("@.service")]
            return subprocess.CompletedProcess(argv, 0, "".join(f"{u} loaded x y\n" for u in listed), "")
        if argv[:3] == [*SUDO, "/usr/bin/systemd-run"]:
            self.calls.append(("deferred", argv[-1]))
            return subprocess.CompletedProcess(argv, 0, "Running timer as unit: run-u1.timer\n", "")
        if argv[:3] == [*SUDO, "/usr/bin/systemctl"] and argv[3] == "restart":
            self.calls.append(("restart", argv[4]))
            self.states[argv[4]] = "failed" if argv[4] in self.die_on_start else "active"
            return subprocess.CompletedProcess(argv, 0, "", "")
        return super().run(argv, **kwargs)

    def _show(self, unit: str) -> str:
        state = self.states.get(unit, "inactive")
        if unit not in self.installed:
            return "LoadState=not-found\nActiveState=inactive\nSubState=dead\nUnitFileState=\n"
        sub = {"active": "running", "failed": "failed"}.get(state, "dead")
        return f"LoadState=loaded\nActiveState={state}\nSubState={sub}\nUnitFileState={self.installed[unit]}\n"


@pytest.fixture
def host(monkeypatch) -> HostStub:
    stub = HostStub()
    stub.installed = {
        "fm-axol-core.service": "static",
        "fm-axol-zed.service": "static",
        "fm-robot-agent.service": "enabled",
        "fm-twin-robot@.service": "disabled",
        "zed_x_daemon.service": "enabled",
    }
    stub.states = {"fm-robot-agent.service": "active", "zed_x_daemon.service": "active"}
    monkeypatch.setattr(subprocess, "run", stub.run)
    return stub


def ask(payload, adapter=None):
    reply = answer(f"{KEY_PREFIX}/{NS}/unit", adapter or FakeAdapter(), NS, payload=json.dumps(payload).encode())
    return reply.ok, json.loads(reply.payload)


# --- naming ------------------------------------------------------------------


@pytest.mark.parametrize(
    ("name", "unit"),
    [
        ("fm-axol-zed", "fm-axol-zed.service"),
        ("fm-axol-zed.service", "fm-axol-zed.service"),
        ("zed_x_daemon", "zed_x_daemon.service"),
        ("nvargus-daemon.service", "nvargus-daemon.service"),
    ],
)
def test_a_name_is_normalised_to_its_service(name, unit):
    assert units.normalise(name) == unit


@pytest.mark.parametrize(
    "name",
    ["sshd", "fm-", "fm-a\n", "FM-axol", "fm-axol;reboot", "fm-twin@x", "fm-a.timer", "../fm-a", "fm-a service", 3, None],
)
def test_a_name_outside_the_allowlist_is_refused(name):
    assert units.normalise(name) is None


def test_the_verb_is_a_write_no_wildcard_reaches():
    assert verb_of(f"{KEY_PREFIX}/{NS}/unit", NS) == "unit"
    assert verb_of(f"{KEY_PREFIX}/*/unit", NS) is None


# --- reads -------------------------------------------------------------------


def test_list_reports_every_fm_service_and_the_camera_daemons_present(host):
    ok, reply = ask({"action": "list"})
    assert ok and reply["ok"] is True
    assert reply["units"] == [
        {"name": "fm-axol-core.service", "active": "inactive", "sub": "dead", "enabled": "static"},
        {"name": "fm-axol-zed.service", "active": "inactive", "sub": "dead", "enabled": "static"},
        {"name": "fm-robot-agent.service", "active": "active", "sub": "running", "enabled": "enabled"},
        {"name": "zed_x_daemon.service", "active": "active", "sub": "running", "enabled": "enabled"},
    ]


def test_a_bare_body_lists(host):
    assert "units" in ask({})[1]


def test_list_takes_no_unit(host):
    ok, reply = ask({"action": "list", "unit": "fm-axol-zed"})
    assert not ok and "takes no unit" in reply["error"]


def test_status_describes_one_unit(host):
    ok, reply = ask({"action": "status", "unit": "fm-robot-agent"})
    assert ok and reply["ok"] is True
    assert {k: reply[k] for k in ("name", "active", "sub", "enabled")} == {
        "name": "fm-robot-agent.service", "active": "active", "sub": "running", "enabled": "enabled",
    }


def test_status_of_a_unit_this_host_lacks_says_not_found(host):
    assert ask({"action": "status", "unit": "fm-nothing"})[1]["enabled"] == "not-found"


def test_a_systemctl_that_will_not_answer_is_a_refusal(host, monkeypatch):
    def hang(argv, **_kwargs):
        raise subprocess.TimeoutExpired(argv, 10)

    monkeypatch.setattr(subprocess, "run", hang)
    ok, reply = ask({"action": "list"})
    assert not ok and "did not answer" in reply["error"]


# --- changes -----------------------------------------------------------------


@pytest.mark.parametrize("action", ["start", "restart"])
def test_a_change_runs_one_unit_under_non_interactive_sudo(host, action):
    ok, reply = ask({"action": action, "unit": "fm-axol-zed"})
    assert ok and reply["ok"] is True and reply["active"] == "active"
    assert [*SUDO, "/usr/bin/systemctl", action, "fm-axol-zed.service"] in host.argv


def test_stop_reports_the_unit_inactive(host):
    host.states["fm-axol-zed.service"] = "active"
    ok, reply = ask({"action": "stop", "unit": "fm-axol-zed"})
    assert ok and reply["ok"] is True and reply["active"] == "inactive"


def test_a_unit_that_dies_after_start_is_not_started(host):
    host.die_on_start.add("fm-axol-zed.service")
    ok, reply = ask({"action": "start", "unit": "fm-axol-zed"})
    assert ok and reply["ok"] is False
    assert reply["message"] == "fm-axol-zed.service is failed after start"


def test_a_failed_start_carries_systemctls_reason(host):
    host.fail_start.add("fm-axol-zed.service")
    assert "failed: Job for" in ask({"action": "start", "unit": "fm-axol-zed"})[1]["message"]


def test_a_camera_daemon_restarts_and_nothing_more(host):
    assert ask({"action": "restart", "unit": "zed_x_daemon"})[1]["ok"] is True
    for action in ("start", "stop"):
        ok, reply = ask({"action": action, "unit": "zed_x_daemon"})
        assert not ok and "takes status, restart only" in reply["error"]


def test_the_agent_refuses_to_stop_itself(host):
    ok, reply = ask({"action": "stop", "unit": "fm-robot-agent"})
    assert not ok and "cannot stop itself" in reply["error"]
    assert host.changes() == []


def test_the_agent_restarts_itself_after_its_reply(host):
    """systemctl restart run in-process would kill the agent before it answered."""
    ok, reply = ask({"action": "restart", "unit": "fm-robot-agent.service"})
    assert ok and reply["ok"] is True and "restarts in 1 s" in reply["message"]
    assert [*SUDO, "/usr/bin/systemd-run", "--on-active=1", "/usr/bin/systemctl", "restart",
            "fm-robot-agent.service"] in host.argv
    assert ("restart", "fm-robot-agent.service") not in host.calls


@pytest.mark.parametrize("body", [{"action": "start"}, {"action": "reboot", "unit": "fm-a"},
                                  {"action": "start", "unit": "sshd"}])
def test_a_malformed_change_is_refused_before_systemctl(host, body):
    ok, _reply = ask(body)
    assert not ok
    assert host.argv == []


def test_an_adapter_refusal_stops_the_change(host):
    class Guarded(FakeAdapter):
        def unit_refusal(self, action, unit):
            return Outcome(ok=False, message=f"no {action} of {unit}")

    ok, reply = ask({"action": "start", "unit": "fm-axol-zed"}, Guarded())
    assert ok and reply == {**reply, "ok": False, "message": "no start of fm-axol-zed.service"}
    assert host.changes() == []


# --- the settle window -------------------------------------------------------


def test_a_start_watches_the_unit_through_the_settle_window(monkeypatch):
    """The BACKLOG bug: a unit that dies a moment after start reported ok."""
    reads = iter(["active", "active", "failed"])
    monkeypatch.setattr(units, "active_state", lambda unit: next(reads))
    monkeypatch.setattr(units, "SETTLE_S", 10)
    monkeypatch.setattr(units.time, "sleep", lambda _s: None)
    assert units.settled_state(CORE_UNIT) == "failed"


def test_a_unit_that_stays_up_settles_active(monkeypatch):
    clock = iter([0.0, 0.0, 1.0, 2.0, 3.0])
    monkeypatch.setattr(units.time, "monotonic", lambda: next(clock))
    monkeypatch.setattr(units.time, "sleep", lambda _s: None)
    monkeypatch.setattr(units, "active_state", lambda unit: "active")
    monkeypatch.setattr(units, "SETTLE_S", 3.0)
    assert units.settled_state(CORE_UNIT) == "active"


def test_the_ros_stack_start_uses_the_settle_window(systemctl, monkeypatch):
    seen = []
    monkeypatch.setattr("fm_robot_agent.axol_ros.settled_state", lambda unit: seen.append(unit) or "failed")
    failure = RosStack().start()
    assert seen == [CORE_UNIT]
    assert failure.startswith(f"{CORE_UNIT} is failed after start")
