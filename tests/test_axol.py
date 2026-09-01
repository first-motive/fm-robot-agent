"""The Axol adapter, against a stub of Almond's FastAPI server."""

from __future__ import annotations

import json
import urllib.error
import urllib.request

import pytest

from fm_robot_agent.axol import (
    KIND,
    MODES,
    RECORD_OPERATION,
    AxolAdapter,
    _frame_to_joint_state,
)
from fm_robot_agent.protocol import AdapterError


class ServerStub:
    """Almond's server, recording what it was asked and answering plausibly."""

    def __init__(self) -> None:
        self.requests: list[tuple[str, str, dict | None]] = []
        self.running_op: str | None = None
        self.state = "connected"
        self.refuse: set[str] = set()

    def handle(self, method: str, path: str, payload: dict | None) -> dict:
        self.requests.append((method, path, payload))
        if path in self.refuse:
            raise urllib.error.HTTPError(path, 409, "Conflict", None, None)
        if path == "/api/robot/status":
            return {"state": self.state, "motors": {"left:SHOULDER_1": {"reachable": True}}}
        if path == "/api/op/status":
            return {
                "running": self.running_op is not None,
                "session": {"id": "s1", "op": self.running_op} if self.running_op else None,
            }
        if path == "/api/op/start":
            self.running_op = payload["op"]
            return {"id": "s2", "op": self.running_op}
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


def test_status_names_the_running_mode_in_fleet_terms(adapter, server):
    adapter.set_mode("policy")
    assert server.running_op == "run-policy"
    assert adapter.status()["mode"] == "policy"


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


# --- recording ---------------------------------------------------------------


def test_record_start_scopes_the_dataset_to_the_configured_owner(adapter, server):
    adapter.record("pick-place", "start")
    _method, _path, payload = server.requests[-1]
    assert payload["args"]["repo_id"] == "axol/pick-place"


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
