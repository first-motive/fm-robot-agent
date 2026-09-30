"""The SO-101 adapter, against a stub of fm-teleop's stack on a real loopback port."""

from __future__ import annotations

import json
import math
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from fm_robot_agent import so101
from fm_robot_agent.cdr import joint_state
from fm_robot_agent.config import MODE, MODE_ALIAS
from fm_robot_agent.protocol import AdapterError
from fm_robot_agent.so101 import KIND, Profile, So101Adapter


class StackStub:
    """The stack's loopback API: what it was asked, and a plausible answer."""

    def __init__(self) -> None:
        self.requests: list[tuple[str, str, dict | None]] = []
        self.mode = "idle"
        self.take: dict | None = None
        self.last_episode: dict | None = None
        self.joints = {"shoulder_pan": 90.0, "gripper": 50.0}
        self.refuse: dict[str, str] = {}

    def status(self) -> dict:
        return {"mode": self.mode, "joints": self.joints, "recording": self.take, "last_episode": self.last_episode}

    def answer(self, method: str, path: str, body: dict | None) -> tuple[int, dict]:
        self.requests.append((method, path, body))
        if path in self.refuse:
            return 409, {"ok": False, "error": self.refuse[path]}
        if path == "/record" and body["action"] == "start":
            self.take = {"dataset": body["dataset"], "task": body["task"]}
            self.mode = "recording"
        elif path == "/record":
            self.last_episode = {"dataset": self.take["dataset"], "episode": 4, "task": self.take["task"]}
            self.take, self.mode = None, "teleop"
        elif path == "/mode":
            self.mode = body["mode"]
        return 200, {"ok": True, **self.status()}


@pytest.fixture
def stack():
    stub = StackStub()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            self._send(200, stub.status())

        def do_POST(self):
            length = int(self.headers.get("Content-Length") or 0)
            self._send(*stub.answer("POST", self.path, json.loads(self.rfile.read(length) or b"{}")))

        def _send(self, code, payload):
            data = json.dumps(payload).encode()
            self.send_response(code)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, *_args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    stub.port = server.server_address[1]
    yield stub
    server.shutdown()


def profile(tmp_path, project="unused") -> Profile:
    return Profile(
        name="fm-rob-03",
        leader_port="serial:LEADER",
        follower_port="serial:FOLLOWER",
        leader_id="fm_rob_03_leader",
        follower_id="fm_rob_03_follower",
        root=str(tmp_path / "lerobot"),
        stack_project=project,
    )


@pytest.fixture
def adapter(tmp_path, stack):
    return So101Adapter(profile(tmp_path), port=stack.port)


@pytest.fixture
def unanswered(tmp_path):
    """An adapter whose stack port answers nothing: bind one, then free it."""
    server = ThreadingHTTPServer(("127.0.0.1", 0), BaseHTTPRequestHandler)
    port = server.server_address[1]
    server.server_close()
    return So101Adapter(profile(tmp_path), port=port)


# --- status -------------------------------------------------------------------


def test_the_adapter_names_its_kind():
    assert So101Adapter.kind == KIND == "so101"


def test_status_reports_the_contract_fields(adapter):
    status = adapter.status()
    for key in ("mode", "modes", "hardware", "recording", "services", "disk", "memory"):
        assert key in status, f"status lacks {key}"
    assert status["modes"] == ["idle", "teleop"]
    assert status["hardware"] == "connected"
    assert status["services"] == [{"name": "fm-teleop-so101", "state": "running"}]


def test_a_take_is_reported_as_recording_in_teleop_mode(adapter, stack):
    stack.mode, stack.take = "recording", {"dataset": "pick", "task": "pick it up"}
    status = adapter.status()
    assert status["mode"] == "teleop", "recording is a take inside teleop, not a mode the picker offers"
    assert status["recording"] is True


def test_a_stack_that_does_not_answer_reads_as_stack_down(unanswered):
    status = unanswered.status()
    assert (status["hardware"], status["mode"], status["recording"]) == ("stack down", None, False)
    assert status["services"] == [{"name": "fm-teleop-so101", "state": "stopped"}]


# --- verbs ----------------------------------------------------------------------


def test_a_mode_goes_to_the_stack(adapter, stack):
    assert adapter.set_mode("teleop").ok
    assert stack.requests[-1] == ("POST", "/mode", {"mode": "teleop"})


def test_a_stack_refusal_surfaces_its_reason(adapter, stack):
    stack.refuse["/record"] = "switch to teleop first; recording never starts motion by itself"
    outcome = adapter.record("pick", "start", task="pick it up")
    assert not outcome.ok
    assert outcome.message == "switch to teleop first; recording never starts motion by itself"


def test_a_take_carries_its_dataset_and_task(adapter, stack):
    assert adapter.record("pick", "start", task="pick up the cube").ok
    assert stack.requests[-1][2] == {"action": "start", "dataset": "pick", "task": "pick up the cube"}


def test_stopping_a_take_names_the_saved_episode(adapter, stack):
    adapter.record("pick", "start", task="pick up the cube")
    outcome = adapter.record("pick", "stop")
    assert outcome.ok
    assert outcome.detail == {"episode": "4"}


def test_a_stop_note_or_episode_check_is_refused_without_asking_the_stack(adapter, stack):
    assert not adapter.record("pick", "stop", note="dropped it").ok
    assert not adapter.record("pick", "stop", episode="3").ok
    assert stack.requests == []


def test_stop_asks_the_stack_for_torque_off(adapter, stack):
    assert adapter.stop().ok
    assert stack.requests[-1][:2] == ("POST", "/stop")


def test_verbs_against_a_silent_stack_raise_rather_than_claim_success(unanswered):
    with pytest.raises(AdapterError):
        unanswered.set_mode("teleop")


def test_config_offers_only_the_mode(adapter):
    [setting] = adapter.config_read()
    assert (setting.key, setting.klass, setting.options) == (MODE_ALIAS, MODE, ("idle", "teleop"))
    assert not adapter.config_write("max_relative_target", "90").ok
    assert not adapter.config_rollback().ok


def test_there_is_no_shared_session_to_pin(adapter):
    assert not adapter.session("set", "pick").ok


def test_an_absent_dataset_lists_no_episodes(adapter):
    assert adapter.episodes("never-recorded") == []


# --- lifecycle -------------------------------------------------------------------


def test_up_with_a_stack_already_answering_starts_nothing(adapter):
    assert adapter.up().message == "stack already running"
    assert adapter._stack is None


def test_up_reports_a_stack_that_exits_and_names_its_log(tmp_path, unanswered, monkeypatch):
    monkeypatch.setattr(Profile, "stack_command", lambda self, port: ["false"])
    outcome = unanswered.up()
    assert not outcome.ok
    assert "stack exited" in outcome.message and "fm-teleop-so101.log" in outcome.message


def test_down_with_nothing_started_and_nothing_answering_is_down(unanswered):
    assert unanswered.down().ok


def test_down_refuses_to_claim_a_stack_it_did_not_start(adapter):
    assert not adapter.down().ok, "a stack started by hand still holds the arms"


# --- telemetry -------------------------------------------------------------------


def test_telemetry_speaks_radians_and_a_unit_gripper(adapter, monkeypatch):
    monkeypatch.setattr(so101.time, "time", lambda: 1000.0)
    topic, payload = next(adapter.telemetry())
    assert topic == "joint_states"
    expected = joint_state(
        stamp_s=1000.0,
        names=["gripper", "shoulder_pan"],
        positions=[0.5, math.radians(90.0)],
        velocities=[0.0, 0.0],
        efforts=[0.0, 0.0],
    )
    assert payload == expected


# --- profile ---------------------------------------------------------------------


@pytest.mark.parametrize("name", ["../etc/passwd", "fm-rob-3", "fm-rob-03/x", "axol"])
def test_a_robot_name_that_is_not_a_fleet_name_is_refused(name):
    with pytest.raises(AdapterError):
        so101.profile_path(name)


def test_a_profile_round_trips_through_the_hosts_config(tmp_path, monkeypatch):
    monkeypatch.setattr(so101.platform, "system", lambda: "Darwin")
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    written = profile(tmp_path, project="/opt/fm/fm-teleop/fm_teleop_so101")
    path = written.write()
    assert path == tmp_path / "fm" / "robots" / "fm-rob-03.json"
    assert Profile.read("fm-rob-03") == written


def test_a_missing_profile_says_how_to_write_one(tmp_path, monkeypatch):
    monkeypatch.setattr(so101.platform, "system", lambda: "Darwin")
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    with pytest.raises(AdapterError, match="fm robot host fm-rob-03 --leader-port"):
        Profile.read("fm-rob-03")


def test_a_profile_missing_a_field_names_it(tmp_path, monkeypatch):
    monkeypatch.setattr(so101.platform, "system", lambda: "Darwin")
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    path = tmp_path / "fm" / "robots" / "fm-rob-03.json"
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps({"kind": "so101", "leader_port": "serial:X"}))
    with pytest.raises(AdapterError, match="follower_port"):
        Profile.read("fm-rob-03")


def test_the_stack_command_is_an_argv_not_a_shell_line(tmp_path):
    argv = profile(tmp_path).stack_command(8790)
    assert argv[:5] == ["uv", "run", "--project", "unused", "fm-teleop-so101"]
    assert argv[argv.index("--owner") + 1] == "fm-rob-03"
