"""The Axol adapter, against a stub of Almond's FastAPI server."""

from __future__ import annotations

import json
import subprocess
import threading
import urllib.error
import urllib.request
from pathlib import Path

import pytest

from fm_robot_agent.axol import (
    ALL_MODES,
    KIND,
    MODES,
    RECORD_OPERATION,
    ROS_CHECK_INTERVAL_S,
    ROS_MODE,
    AxolAdapter,
    _frame_to_joint_state,
)
from fm_robot_agent.axol_ros import CAMERA_UNIT, CORE_UNIT, ROS_UNIT, SUDOERS_RULE
from fm_robot_agent.config import MODE, MODE_ALIAS, MOTION, TUNING
from fm_robot_agent.protocol import AdapterError


class ServerStub:
    """Almond's server, recording what it was asked and answering plausibly."""

    def __init__(self) -> None:
        self.requests: list[tuple[str, str, dict | None]] = []
        self.running_op: str | None = None
        # Recorded because the real server refuses a start whose args are wrong,
        # and a fake that drops them accepted an argument-less `run-policy` — the
        # assumption that cost a bench session.
        self.started_args: dict = {}
        self.state = "connected"
        self.refuse: set[str] = set()
        self.motors: dict | list | None = {"left:SHOULDER_1": {"reachable": True}}

    exited_session: dict | None = None

    def handle(self, method: str, path: str, payload: dict | None) -> dict:
        self.requests.append((method, path, payload))
        if path in self.refuse:
            raise urllib.error.HTTPError(path, 409, "Conflict", None, None)
        if path == "/api/robot/status":
            return {"state": self.state, "motors": self.motors}
        if path == "/api/op/status":
            if self.running_op is not None:
                return {"running": True, "session": {"id": "s1", "command": self.running_op}}
            return {"running": False, "session": self.exited_session}
        if path == "/api/op/start":
            self.running_op = payload["op"]
            self.started_args = dict(payload.get("args") or {})
            return {"id": "s2", "command": self.running_op}
        if path == "/api/op/stop":
            if self.running_op is None:
                raise urllib.error.HTTPError(path, 404, "no operation running", None, None)
            self.running_op = None
            return {"id": "s1"}
        if path == "/api/robot/connect":
            self.state = "connected"
            return {"state": self.state}
        if path == "/api/robot/disconnect":
            self.state = "disconnected"
            return {"state": self.state}
        return {}

    def paths(self) -> list[str]:
        return [path for _method, path, _payload in self.requests]


class FakeResponse:
    """What `urlopen` hands back: a context manager over the response bytes."""

    def __init__(self, payload: dict) -> None:
        self._body = json.dumps(payload).encode()

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        return False

    def read(self) -> bytes:
        return self._body


@pytest.fixture
def server(monkeypatch):
    """Patch at the socket boundary, so the adapter's own error handling runs."""
    stub = ServerStub()

    def fake_urlopen(request, timeout=None, context=None):
        body = request.data
        payload = json.loads(body) if body else None
        return FakeResponse(stub.handle(request.get_method(), request.selector, payload))

    monkeypatch.setattr(urllib.request, "urlopen", fake_urlopen)
    return stub


@pytest.fixture(autouse=True)
def units(systemctl):
    """Every test here reads the fm-axol units, so none reaches the host's systemctl."""
    return systemctl


@pytest.fixture
def adapter(server):
    return AxolAdapter(base_url="http://localhost:8001", dataset_owner="axol")


# --- identity ----------------------------------------------------------------


def test_the_adapter_names_the_card_kind():
    assert AxolAdapter.kind == KIND == "axol"


@pytest.mark.parametrize("owner", ["../../..", "axol/nested", "Axol", "axol "])
def test_a_dataset_owner_that_is_a_path_is_refused(owner):
    """The owner becomes a directory component, so it is held to a name's shape."""
    with pytest.raises(AdapterError):
        AxolAdapter(dataset_owner=owner)


# --- status ------------------------------------------------------------------


def test_status_reports_the_contract_fields(adapter):
    reported = adapter.status()
    assert set(reported) >= {"mode", "hardware", "recording", "services", "disk"}
    assert reported["hardware"] == "connected"
    assert reported["mode"] is None
    assert reported["recording"] is False


@pytest.mark.parametrize(
    ("detail", "count"),
    [
        ({"left:SHOULDER_1": {"reachable": True}}, 1),
        ([{"arm": "left", "joint": "SHOULDER_1"}, {"arm": "left", "joint": "ELBOW"}], 2),
        (None, None),
    ],
)
def test_status_counts_motors_whatever_shape_almond_sends(adapter, server, detail, count):
    server.motors = detail
    reported = adapter.status()
    assert reported["motors"] == count
    assert reported["motor_states"] == detail


def test_status_names_the_running_mode_in_fleet_terms(adapter, server):
    adapter.set_mode("teleop")
    assert server.running_op == "teleop"
    assert adapter.status()["mode"] == "teleop"


def test_policy_mode_refuses_without_what_run_policy_requires(adapter, server):
    """Almond answers a missing field with a bare 500 and no reason on the wire.

    `run-policy` needs `policy_path`, `policy_type` and `task`, none of them a
    setting the robot stores. This test used to start `policy` with no arguments
    at all and pass, because the fake server accepted them — the assumption was
    wrong on hardware, where the reason reached only the robot's journal.
    """
    outcome = adapter.set_mode("policy")

    assert outcome.ok is False
    for field in ("policy_path", "policy_type", "task"):
        assert field in outcome.message
    assert server.running_op is None, "a refused mode must not start an operation"


def test_policy_mode_starts_with_its_arguments(adapter, server):
    outcome = adapter.set_mode(
        "policy",
        {
            "policy_path": "first-motive/policy-act-1",
            "policy_type": "act",
            "task": "open the cup",
        },
    )

    assert outcome.ok is True
    assert server.running_op == "run-policy"
    assert server.started_args["policy_path"] == "first-motive/policy-act-1"
    assert server.started_args["task"] == "open the cup"


def test_a_partly_supplied_policy_mode_names_only_what_is_missing(adapter, server):
    outcome = adapter.set_mode("policy", {"policy_type": "act"})

    assert outcome.ok is False
    assert "policy_path" in outcome.message
    assert "task" in outcome.message
    assert "policy_type" not in outcome.message


def test_a_session_that_exited_is_not_a_mode(adapter, server):
    """The Axol keeps its last session in status; reading the name alone lies."""
    server.exited_session = {"id": "s1", "command": "teleop", "status": "exited"}
    reported = adapter.status()
    assert reported["mode"] is None
    assert reported["recording"] is False


def test_status_reports_recording_only_for_the_record_operation(adapter, server):
    adapter.record("pick-place", "start")
    assert server.running_op == RECORD_OPERATION
    assert adapter.status()["recording"] is True


# --- link --------------------------------------------------------------------


def test_up_connects_without_choosing_can_interfaces(adapter, server):
    """Re-pointing a CAN bus is a bench decision, never a fleet one."""
    adapter.up()
    _method, path, payload = server.requests[-1]
    assert path == "/api/robot/connect"
    assert payload == {}


def test_down_disconnects(adapter, server):
    adapter.down()
    assert server.state == "disconnected"


# --- modes -------------------------------------------------------------------


def test_every_fleet_mode_maps_to_an_operation():
    assert set(MODES) == {"teleop", "gravity-comp", "policy"}


def test_an_unknown_mode_is_refused_without_touching_the_robot(adapter, server):
    outcome = adapter.set_mode("dance")
    assert outcome.ok is False
    assert server.requests == []


def test_switching_mode_stops_what_was_running_first(adapter, server):
    adapter.set_mode("teleop")
    server.requests.clear()
    adapter.set_mode("gravity-comp")
    assert server.paths()[0] == "/api/op/stop"
    assert server.running_op == "gravity-comp"


def test_switching_mode_with_nothing_running_still_starts(adapter, server):
    """`/api/op/stop` answers 404 when idle; that is not a failure to report."""
    assert adapter.set_mode("teleop").ok is True
    assert server.running_op == "teleop"


# --- ros mode ----------------------------------------------------------------


def test_ros_mode_disconnects_almond_then_starts_core_then_container_then_cameras(adapter, server, units):
    outcome = adapter.set_mode(ROS_MODE)

    assert outcome.ok is True
    assert server.state == "disconnected", "Almond's link must let go of the buses first"
    assert units.changes() == [("start", CORE_UNIT), ("start", ROS_UNIT), ("start", CAMERA_UNIT)]
    reported = adapter.status()
    assert reported["mode"] == ROS_MODE
    assert ROS_MODE in reported["modes"]
    assert {"name": CORE_UNIT, "state": "active"} in reported["services"]


@pytest.mark.parametrize("operation", ["teleop", "gravity-comp"])
def test_ros_mode_refuses_while_an_almond_operation_runs(adapter, server, units, operation):
    adapter.set_mode(operation)

    outcome = adapter.set_mode(ROS_MODE)

    assert outcome.ok is False
    assert "Almond operation is running" in outcome.message
    assert units.changes() == []
    assert server.running_op == operation


def test_a_camera_failure_still_enters_ros_mode_and_says_so(adapter, units):
    """Motion needs no camera; the reply and status still name the failure."""
    units.fail_start.add(CAMERA_UNIT)

    outcome = adapter.set_mode(ROS_MODE)

    assert outcome.ok is True
    assert CAMERA_UNIT in outcome.message and "without cameras" in outcome.message
    assert CAMERA_UNIT in outcome.detail["camera_failure"]
    reported = adapter.status()
    assert reported["mode"] == ROS_MODE
    assert {"name": CAMERA_UNIT, "state": "inactive"} in reported["services"]


def test_a_camera_that_dies_on_start_is_reported_in_status(adapter, units):
    units.die_on_start.add(CAMERA_UNIT)

    outcome = adapter.set_mode(ROS_MODE)

    assert outcome.ok is True and f"{CAMERA_UNIT} is failed after start" in outcome.message
    assert {"name": CAMERA_UNIT, "state": "failed"} in adapter.status()["services"]


def test_a_failed_ros_start_never_starts_the_cameras(adapter, units):
    units.fail_start.add(ROS_UNIT)
    adapter.set_mode(ROS_MODE)
    assert ("start", CAMERA_UNIT) not in units.changes()


def test_ros_mode_refuses_while_recording(adapter, server, units):
    adapter.record("pick-place", "start")
    assert adapter.set_mode(ROS_MODE).ok is False
    assert units.changes() == []


def test_a_failed_ros_start_rolls_both_units_back(adapter, units):
    units.fail_start.add(ROS_UNIT)

    outcome = adapter.set_mode(ROS_MODE)

    assert outcome.ok is False
    assert "both units stopped" in outcome.message
    assert units.changes()[-2:] == [("stop", ROS_UNIT), ("stop", CORE_UNIT)]
    assert adapter.status()["mode"] is None


def test_a_sudo_refusal_fails_ros_mode_with_a_clear_reason(adapter, units):
    units.sudo_refusal = "sudo: a password is required\n"

    outcome = adapter.set_mode(ROS_MODE)

    assert outcome.ok is False
    assert "sudo refused" in outcome.message and SUDOERS_RULE in outcome.message
    assert adapter.status()["mode"] is None


def test_ros_mode_takes_no_arguments(adapter, units):
    assert adapter.set_mode(ROS_MODE, {"policy_path": "x"}).ok is False
    assert units.changes() == []


def test_ros_mode_twice_starts_nothing_twice(adapter, units):
    adapter.set_mode(ROS_MODE)
    units.calls.clear()
    assert adapter.set_mode(ROS_MODE).ok is True
    assert units.changes() == []


@pytest.mark.parametrize(
    "request_operation",
    [
        lambda a: a.set_mode("teleop"),
        lambda a: a.set_mode("gravity-comp"),
        lambda a: a.set_mode("policy", {"policy_path": "p", "policy_type": "act", "task": "t"}),
        lambda a: a.record("pick-place", "start"),
        lambda a: a.config_write(MODE_ALIAS, "teleop"),
        lambda a: a.up(),
    ],
    ids=["teleop", "gravity-comp", "policy", "record", "config-mode", "up"],
)
def test_almond_operations_refuse_while_ros_mode_is_on(adapter, server, request_operation):
    adapter.set_mode(ROS_MODE)
    server.requests.clear()

    outcome = request_operation(adapter)

    assert outcome.ok is False
    assert "mode ros is on" in outcome.message
    assert not {"/api/op/start", "/api/robot/connect"} & set(server.paths())


def test_stop_ends_ros_mode_cameras_first_and_frees_almond(adapter, server, units):
    adapter.set_mode(ROS_MODE)
    units.calls.clear()

    outcome = adapter.stop()

    assert outcome.ok is True
    assert units.changes() == [("stop", CAMERA_UNIT), ("stop", ROS_UNIT), ("stop", CORE_UNIT)]
    assert adapter.set_mode("teleop").ok is True


def test_stop_releases_cameras_left_running_after_the_core_died(adapter, server, units):
    """Almond cannot stream a camera the ZED driver still holds."""
    adapter.set_mode(ROS_MODE)
    units.states[CORE_UNIT] = units.states[ROS_UNIT] = "failed"
    units.calls.clear()

    assert adapter.stop().ok is True
    assert ("stop", CAMERA_UNIT) in units.changes()
    assert units.states[CAMERA_UNIT] == "inactive"


def test_a_stock_axol_stop_touches_no_unit(adapter, units):
    assert adapter.stop().ok is True
    assert units.changes() == []


def test_down_ends_ros_mode(adapter, units):
    adapter.set_mode(ROS_MODE)
    assert adapter.down().ok is True
    assert adapter.status()["mode"] is None


def test_a_ros_stack_that_will_not_stop_fails_the_stop(adapter, server, units):
    adapter.set_mode(ROS_MODE)
    units.fail_stop.add(CORE_UNIT)

    outcome = adapter.stop()

    assert outcome.ok is False
    assert CORE_UNIT in outcome.message
    assert server.state == "disconnected"


def test_stop_on_a_stock_axol_touches_no_unit(adapter, units):
    """A unit that is not loaded fails to stop; an idle stack is never asked to."""
    assert adapter.stop().ok is True
    assert units.changes() == []


# --- recording ---------------------------------------------------------------


def test_record_start_scopes_the_dataset_to_the_configured_owner(adapter, server):
    adapter.record("pick-place", "start")
    _method, _path, payload = server.requests[-1]
    assert payload["args"]["repo_id"] == "axol/pick-place"


def test_record_refuses_a_task_collect_data_has_no_field_for(adapter, server):
    """`run-policy` names its task field; `collect-data` never has, so this refuses."""
    outcome = adapter.record("pick-place", "start", "put the nuts in the bag")

    assert outcome.ok is False
    assert "fm policy dataset relabel" in outcome.message
    assert server.running_op is None


def test_record_stop_stops_the_operation(adapter, server):
    adapter.record("pick-place", "start")
    adapter.record("pick-place", "stop")
    assert server.running_op is None


# --- stop --------------------------------------------------------------------


def test_stop_disconnects_the_motors(adapter, server):
    adapter.set_mode("teleop")
    outcome = adapter.stop()
    assert outcome.detail["state"] == "disconnected"
    assert server.running_op is None
    assert server.state == "disconnected"


def test_a_refusing_server_surfaces_the_reason(adapter, server):
    server.refuse.add("/api/robot/connect")
    with pytest.raises(AdapterError):
        adapter.up()


# --- episodes ----------------------------------------------------------------


def test_episodes_read_the_lerobot_index(tmp_path, monkeypatch, server):
    monkeypatch.setenv("HF_LEROBOT_HOME", str(tmp_path))
    root = tmp_path / "axol" / "pick-place"
    (root / "meta").mkdir(parents=True)
    (root / "data" / "chunk-000").mkdir(parents=True)
    (root / "data" / "chunk-000" / "episode_000000.parquet").write_bytes(b"x" * 2048)
    (root / "meta" / "episodes.jsonl").write_text(
        '{"episode_index": 0, "length": 120, "tasks": ["pick"]}\n'
        "not json\n"
        '{"episode_index": 1, "length": 90, "tasks": ["place"]}\n',
        encoding="utf-8",
    )
    listed = AxolAdapter(dataset_owner="axol").episodes("pick-place")
    assert [e["slug"] for e in listed] == ["000000", "000001"]
    assert listed[0]["size_kb"] == 2
    assert listed[0]["length"] == 120


def test_an_absent_dataset_lists_nothing(tmp_path, monkeypatch):
    monkeypatch.setenv("HF_LEROBOT_HOME", str(tmp_path))
    assert AxolAdapter().episodes("no-such-dataset") == []


def test_v3_episodes_use_shared_metadata_and_require_dataset_copy(tmp_path, monkeypatch):
    import pyarrow as pa
    import pyarrow.parquet as pq

    monkeypatch.setenv("HF_LEROBOT_HOME", str(tmp_path))
    root = tmp_path / "axol" / "pick"
    metadata = root / "meta" / "episodes" / "chunk-000"
    metadata.mkdir(parents=True)
    (root / "meta" / "info.json").write_text('{"codebase_version":"v3.0","total_episodes":2}')
    pq.write_table(pa.Table.from_pylist([
        {"episode_index": 0, "length": 120, "tasks": ["pick"], "data/file_index": 0},
        {"episode_index": 1, "length": 90, "tasks": ["place"], "data/file_index": 0},
    ]), metadata / "file-000.parquet")
    episodes = AxolAdapter(dataset_owner="axol").episodes("pick")
    assert [row["slug"] for row in episodes] == ["000000", "000001"]
    assert all(row["copy_unit"] == "dataset" and row["size_kb"] is None for row in episodes)
    assert episodes[1]["length"] == 90
    (root / "meta" / "info.json").write_text('{"codebase_version":"v99.0"}')
    with pytest.raises(AdapterError, match="unsupported LeRobot"):
        AxolAdapter(dataset_owner="axol").episodes("pick")


# --- telemetry ---------------------------------------------------------------


def test_a_frame_becomes_a_joint_state_with_a_stable_joint_order():
    """Motor keys are sorted: the sampler's dict order is not a joint order."""
    encoded = _frame_to_joint_state(
        {"type": "frame", "t": 1.0, "m": {"right:ELBOW": [4.0, 5.0, 6.0], "left:SHOULDER_1": [1.0, 2.0, 3.0]}}
    )
    assert b"left:SHOULDER_1\x00" in encoded
    assert encoded.index(b"left:SHOULDER_1") < encoded.index(b"right:ELBOW")


def test_a_short_sample_is_padded_rather_than_dropped():
    """A motor reporting position only still belongs in the message."""
    encoded = _frame_to_joint_state({"t": 1.0, "m": {"left:SHOULDER_1": [1.0]}})
    assert b"left:SHOULDER_1\x00" in encoded


def test_a_non_numeric_sample_is_dropped_rather_than_killing_the_frame():
    encoded = _frame_to_joint_state(
        {"t": 1.0, "m": {"left:SHOULDER_1": "not a number", "right:ELBOW": [1.0, 2.0, 3.0]}}
    )
    assert b"right:ELBOW\x00" in encoded
    assert b"left:SHOULDER_1" not in encoded


def test_a_sample_holding_a_string_is_dropped():
    encoded = _frame_to_joint_state({"t": 1.0, "m": {"left:SHOULDER_1": ["nope", 2.0, 3.0]}})
    assert b"left:SHOULDER_1" not in encoded


def test_an_empty_frame_still_encodes():
    assert _frame_to_joint_state({"t": 1.0, "m": {}})[:4] == b"\x00\x01\x00\x00"


class SocketStub:
    """A telemetry socket that replays a fixed script of messages."""

    def __init__(self, messages) -> None:
        self.messages = messages
        self.closed = False

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.closed = True
        return False

    def __iter__(self):
        return iter(self.messages)


def patch_socket(monkeypatch, messages):
    """Stand in for `websockets.sync.client.connect`, which the adapter imports lazily."""
    import sys
    import types

    stub = SocketStub(messages)
    opened = {}

    def connect(url, open_timeout=None, ssl=None):
        opened["url"] = url
        opened["ssl"] = ssl
        return stub

    module = types.ModuleType("websockets.sync.client")
    module.connect = connect
    package = types.ModuleType("websockets.sync")
    package.client = module
    root = types.ModuleType("websockets")
    root.sync = package
    monkeypatch.setitem(sys.modules, "websockets", root)
    monkeypatch.setitem(sys.modules, "websockets.sync", package)
    monkeypatch.setitem(sys.modules, "websockets.sync.client", module)
    return opened


def test_telemetry_opens_the_axol_socket_as_a_secure_websocket(monkeypatch):
    opened = patch_socket(monkeypatch, [])
    list(AxolAdapter().telemetry())
    assert opened["url"] == "wss://localhost:8001/api/telemetry/ws"


def test_a_plain_http_base_url_still_opens_an_unencrypted_socket(monkeypatch):
    opened = patch_socket(monkeypatch, [])
    list(AxolAdapter(base_url="http://localhost:8001").telemetry())
    assert opened["url"] == "ws://localhost:8001/api/telemetry/ws"
    assert opened["ssl"] is None


def test_telemetry_forwards_only_motor_frames(monkeypatch):
    """The socket also carries slow sweeps and link-state messages; neither is a JointState."""
    patch_socket(
        monkeypatch,
        [
            json.dumps({"type": "state", "state": "busy"}),
            json.dumps({"type": "slow", "t": 1.0, "m": {"left:SHOULDER_1": {"temperature": 40}}}),
            "not json",
            json.dumps({"type": "frame", "t": 1.0, "m": {"left:SHOULDER_1": [1.0, 2.0, 3.0]}}),
        ],
    )
    sent = list(AxolAdapter().telemetry())
    assert len(sent) == 1
    topic, payload = sent[0]
    assert topic == "joint_states"
    assert b"left:SHOULDER_1\x00" in payload


def test_telemetry_drops_frames_above_the_ceiling_rather_than_buffering(monkeypatch):
    """Two frames in the same instant: the second is dropped, never queued."""
    patch_socket(
        monkeypatch,
        [json.dumps({"type": "frame", "t": 1.0, "m": {"left:SHOULDER_1": [1.0, 2.0, 3.0]}})] * 5,
    )
    assert len(list(AxolAdapter().telemetry())) == 1


def test_telemetry_forwards_again_once_the_gap_has_passed(monkeypatch):
    import fm_robot_agent.axol as axol_module

    patch_socket(
        monkeypatch,
        [json.dumps({"type": "frame", "t": 1.0, "m": {"left:SHOULDER_1": [1.0, 2.0, 3.0]}})] * 3,
    )
    clock = iter([100.0, 200.0, 300.0, 400.0])
    monkeypatch.setattr(axol_module.time, "time", lambda: next(clock))
    assert len(list(AxolAdapter().telemetry())) == 3


def frames_while(units, states):
    """One motor frame per entry, each sent with the core in that state."""
    frame = json.dumps({"type": "frame", "t": 1.0, "m": {"left:SHOULDER_1": [1.0, 2.0, 3.0]}})
    for state in states:
        units.states[CORE_UNIT] = state
        yield frame


def test_telemetry_yields_joint_states_to_the_ros_graph(monkeypatch, units):
    """While ros mode runs, the ROS graph's broadcaster is the key's one source."""
    import fm_robot_agent.axol as axol_module

    patch_socket(monkeypatch, frames_while(units, ["active"] * 3))
    clock = iter([100.0, 200.0, 300.0])
    monkeypatch.setattr(axol_module.time, "time", lambda: next(clock))
    assert list(AxolAdapter().telemetry()) == []


def test_telemetry_resumes_once_ros_mode_ends(monkeypatch, units):
    import fm_robot_agent.axol as axol_module

    patch_socket(monkeypatch, frames_while(units, ["active", "active", "inactive", "inactive"]))
    clock = iter([100.0, 200.0, 300.0, 400.0])
    monkeypatch.setattr(axol_module.time, "time", lambda: next(clock))
    assert len(list(AxolAdapter().telemetry())) == 2


def test_telemetry_reads_the_units_at_most_once_per_interval(monkeypatch, units):
    """A state read is a subprocess per unit; a frame rate of them is not free."""
    import fm_robot_agent.axol as axol_module

    patch_socket(monkeypatch, frames_while(units, ["inactive"] * 4))
    clock = iter([100.0, 100.0 + ROS_CHECK_INTERVAL_S / 2, 100.0 + ROS_CHECK_INTERVAL_S, 100.0 + ROS_CHECK_INTERVAL_S * 1.5])
    monkeypatch.setattr(axol_module.time, "time", lambda: next(clock))
    list(AxolAdapter().telemetry())
    assert units.calls.count(("is-active", CORE_UNIT)) == 2


# --- TLS ---------------------------------------------------------------------


def test_the_axol_is_reached_over_https_by_default():
    """Almond's server presents a self-signed certificate on its own loopback."""
    assert AxolAdapter().base_url == "https://localhost:8001"


def test_loopback_certificates_are_not_verified():
    """Self-signed by design, on a connection that never leaves the host."""
    import ssl as ssl_module

    from fm_robot_agent.axol import _ssl_context

    context = _ssl_context("https://localhost:8001")
    assert context.check_hostname is False
    assert context.verify_mode == ssl_module.CERT_NONE


@pytest.mark.parametrize("url", ["https://axol.almond.bot", "wss://axol.almond.bot/x"])
def test_an_off_host_certificate_is_verified_normally(url):
    """Dropping verification is a loopback concession, not a habit."""
    import ssl as ssl_module

    context = _ssl_context_for(url)
    assert context.check_hostname is True
    assert context.verify_mode == ssl_module.CERT_REQUIRED


def _ssl_context_for(url):
    from fm_robot_agent.axol import _ssl_context

    return _ssl_context(url)


def test_plain_http_gets_no_tls_context():
    from fm_robot_agent.axol import _ssl_context

    assert _ssl_context("http://localhost:8001") is None


@pytest.mark.parametrize(
    "host",
    ["localhost", "127.0.0.1", "127.0.0.2", "[::1]", "[0:0:0:0:0:0:0:1]"],
)
def test_every_spelling_of_this_machine_is_loopback(host):
    """A spelling match would demand a certificate nobody can issue for it."""
    from fm_robot_agent.axol import _loopback

    assert _loopback(f"https://{host}:8001") is True


@pytest.mark.parametrize("host", ["axol.almond.bot", "192.168.1.30", "[2001:db8::1]"])
def test_an_off_host_address_is_not_loopback(host):
    from fm_robot_agent.axol import _loopback

    assert _loopback(f"https://{host}:8001") is False


def test_recording_is_true_while_the_record_operation_runs(adapter, server):
    """This read False for every recording until the field name was corrected.

    The asymmetry that hid it: `/api/op/start` takes `op`, and the session it
    returns spells the same value `command` (Session.to_dict in Almond's
    manager). Reading `op` off a session compares against a field that is never
    there, so the answer was always False and never raised.
    """
    adapter.record("pick-place", "start")
    assert server.running_op == RECORD_OPERATION
    assert adapter.status()["recording"] is True


# --- config ------------------------------------------------------------------

#: Almond's own `GET /api/settings` reply, serialized by its own settings module
#: rather than written here. A stub shaped like the caller's expectation proves
#: only that the caller agrees with itself.
ALMOND_SETTINGS = json.loads(
    (Path(__file__).parent / "data" / "almond-settings.json").read_text(encoding="utf-8")
)


@pytest.fixture
def settings_server(server):
    """The stub, answering the settings surface out of Almond's own snapshot."""
    stored = {"values": {}, "cameras": None, "advanced": {}}
    real_handle = server.handle

    def handle(method, path, payload):
        server.requests.append((method, path, payload))
        if path == "/api/settings" and method == "GET":
            return {**ALMOND_SETTINGS, **stored}
        if path == "/api/settings":
            for section in ("values", "advanced"):
                stored[section] = {**stored[section], **(payload.get(section) or {})}
            return dict(stored)
        server.requests.pop()
        return real_handle(method, path, payload)

    server.handle = handle
    server.stored = stored
    return server


def settings(adapter) -> dict:
    return {setting.key: setting for setting in adapter.config_read()}


def test_config_read_forwards_almonds_own_categories(adapter, settings_server):
    listed = settings(adapter)
    assert listed["robot.left_stiffness"].klass == MOTION
    assert listed["recording.fps"].klass == TUNING
    assert listed["kinematics.pos_weight"].klass == MOTION
    assert listed["inference.server_host"].klass == TUNING


def test_config_read_carries_the_vendors_defaults_and_help(adapter, settings_server):
    stiffness = settings(adapter)["robot.left_stiffness"]
    assert stiffness.value == 0.5
    assert "stiffness" in stiffness.help


def test_config_read_carries_the_stored_value_over_the_default(adapter, settings_server):
    settings_server.stored["values"]["recording.fps"] = 30
    assert settings(adapter)["recording.fps"].value == 30


def test_the_mode_is_a_config_key_like_any_other(adapter, settings_server):
    mode = settings(adapter)[MODE_ALIAS]
    assert mode.klass == MODE
    assert mode.options == ALL_MODES


def test_a_tuning_setting_is_written_through(adapter, settings_server):
    outcome = adapter.config_write("recording.fps", "30")
    assert outcome.ok is True
    assert settings_server.stored["values"]["recording.fps"] == 30


def test_a_value_lands_in_the_type_the_vendor_declared(adapter, settings_server):
    adapter.config_write("robot.left_stiffness", "0.8")
    adapter.config_write("recording.observe_torques", "true")
    assert settings_server.stored["values"]["robot.left_stiffness"] == 0.8
    assert settings_server.stored["values"]["recording.observe_torques"] is True


def test_a_value_the_key_cannot_hold_is_refused(adapter, settings_server):
    assert adapter.config_write("recording.fps", "quickly").ok is False
    assert adapter.config_write("recording.observe_torques", "maybe").ok is False
    assert settings_server.stored["values"] == {}


def test_a_setting_this_robot_does_not_have_is_refused(adapter, settings_server):
    assert adapter.config_write("robot.third_arm_stiffness", "1").ok is False


def test_a_motion_setting_is_refused_while_an_operation_runs(adapter, settings_server):
    settings_server.running_op = "teleop"
    assert adapter.config_write("robot.left_stiffness", "0.8").ok is False
    assert settings_server.stored["values"] == {}


def test_a_motion_setting_is_written_once_the_robot_is_idle(adapter, settings_server):
    assert adapter.config_write("robot.left_stiffness", "0.8").ok is True


def test_a_tuning_setting_is_written_while_an_operation_runs(adapter, settings_server):
    """Idle is a guard on motion, not a mood: an fps change interrupts nothing."""
    settings_server.running_op = "teleop"
    assert adapter.config_write("recording.fps", "30").ok is True


# --- the advanced tree -------------------------------------------------------


def test_an_advanced_key_is_passed_through_by_exact_key(adapter, settings_server):
    assert adapter.config_write("axol.left.shoulder_1.kp", "45").ok is True
    assert settings_server.stored["advanced"]["axol.left.shoulder_1.kp"] == 45


def test_an_advanced_key_is_refused_while_an_operation_runs(adapter, settings_server):
    """The whole tree is guarded as motion, because nobody here has read it."""
    settings_server.running_op = "teleop"
    assert adapter.config_write("axol.left.shoulder_1.kp", "45").ok is False


def test_a_key_in_no_advanced_section_is_refused(adapter, settings_server):
    assert adapter.config_write("firmware.left.flash", "1").ok is False


def test_a_stored_advanced_value_is_listed_as_motion(adapter, settings_server):
    settings_server.stored["advanced"]["axol.left.shoulder_1.kp"] = 45
    assert settings(adapter)["axol.left.shoulder_1.kp"].klass == MOTION


def test_the_mode_alias_starts_the_operation_it_names(adapter, settings_server):
    assert adapter.config_write(MODE_ALIAS, "teleop").ok is True
    assert settings_server.running_op == "teleop"


def test_rollback_says_the_axol_journals_nothing(adapter, settings_server):
    assert adapter.config_rollback().ok is False


# --- what the server will actually accept ------------------------------------


def test_starting_an_operation_sends_no_camera_spec(adapter, server):
    """Almond types `cameras` as a dict; a list is refused 422 and nothing runs.

    Omitting it means "use the camera spec this robot already has", which is the
    right answer for the fleet: choosing cameras is a bench decision made in the
    Axol's own UI.
    """
    adapter.set_mode("teleop")
    payload = next(p for m, path, p in server.requests if path == "/api/op/start")
    assert "cameras" not in payload


def test_recording_sends_no_camera_spec(adapter, server):
    adapter.record("pick-place", "start")
    payload = next(p for m, path, p in server.requests if path == "/api/op/start")
    assert "cameras" not in payload
    assert payload["args"]["repo_id"] == "axol/pick-place"


def test_there_is_no_collection_loop(adapter, server):
    """The unattended loop is the Anvil's; an Axol refuses without starting anything."""
    outcome = adapter.collect("start", {"object": "can"})
    assert not outcome.ok and "no collection loop" in outcome.message
    assert server.running_op is None


def test_there_is_no_perception_node(adapter, server):
    outcome = adapter.perception_debug(True)
    assert not outcome.ok and "no perception node" in outcome.message



def test_a_bus_check_and_its_action_run_under_one_lock(adapter, units, monkeypatch):
    """Queries run on their own threads; a check released before its action is a race."""
    entered = threading.Event()
    release = threading.Event()

    def slow_start(argv, **kwargs):
        if argv[-2:] == ["start", CORE_UNIT]:
            entered.set()
            release.wait(5)
        return units.run(argv, **kwargs)

    monkeypatch.setattr(subprocess, "run", slow_start)
    starter = threading.Thread(target=adapter.set_mode, args=(ROS_MODE,))
    starter.start()
    assert entered.wait(5)
    results = []
    racer = threading.Thread(target=lambda: results.append(adapter.up()))
    racer.start()
    racer.join(0.2)
    assert racer.is_alive(), "up must wait for the ros start in flight"
    release.set()
    starter.join(5)
    racer.join(5)
    assert results[0].ok is False and "mode ros is on" in results[0].message
