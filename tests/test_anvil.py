"""The Anvil adapter, against a compose stub and a webapp stub.

Both mocks sit at a system boundary — the ``docker`` subprocess and the webapp's
socket — so what the tests exercise is the adapter's own behaviour and nothing
about how it is wired together.
"""

from __future__ import annotations

import json
import subprocess

import pytest

from fm_robot_agent.anvil import KIND, MODE_KEY, AnvilAdapter
from fm_robot_agent.protocol import AdapterError

RECORDING_RESPONSE = (
    "response:\n"
    "anvil_msgs.srv.GetRecordingStatus_Response("
    "status=anvil_msgs.msg.RecordingStatus(is_recording=False))\n"
)
TOPICS_RESPONSE = "response:\nGetTopics_Response(topics=['/joint_states', '/cam_wrist_l/image_raw'])\n"
COMPOSE_PS = json.dumps({"Service": "ros2", "State": "running", "Status": "Up 2 hours"})


class WebappStub:
    """The webapp's tRPC surface, recording what it was asked."""

    def __init__(self) -> None:
        self.sessions = [{"id": 7, "slug": "grocery-sort-v1", "topics": []}]
        self.calls: list[tuple] = []

    def first_event(self, path, payload=None):
        self.calls.append(("subscription", path, payload))
        return {"sessions": self.sessions}

    def call(self, method, path, payload=None):
        self.calls.append((method, path, payload))
        if path == "recording.start":
            return {"episodeId": 42}
        if path == "recording.stop":
            return {"id": 42}
        if path == "recording.sessions.create":
            created = {"id": 8, "slug": payload["slug"], "topics": payload["topics"]}
            self.sessions.append(created)
            return created
        return {}


@pytest.fixture
def loader(tmp_path):
    """A loader directory shaped like the one on the devbox."""
    (tmp_path / "config").mkdir()
    (tmp_path / "config" / "openarm_v2_quest_teleop.yaml").write_text("", encoding="utf-8")
    (tmp_path / "config" / "openarm_v2_leader_only.yaml").write_text("", encoding="utf-8")
    (tmp_path / "data" / "recordings" / "grocery-sort-v1" / "0000").mkdir(parents=True)
    (tmp_path / "data" / "recordings" / "grocery-sort-v1" / "0000" / "metadata.yaml").write_text(
        "task: pick\n", encoding="utf-8"
    )
    (tmp_path / ".env.config").write_text(
        f"ROS_DOMAIN_ID=1\n{MODE_KEY}=openarm_v2_leader_only.yaml\nENABLE_CYCLONEDDS=true\n",
        encoding="utf-8",
    )
    return tmp_path


@pytest.fixture
def adapter(loader, monkeypatch):
    robot = AnvilAdapter(loader_dir=loader)
    robot.webapp = WebappStub()

    def fake_run(argv, **kwargs):
        joined = " ".join(argv)
        if "ps" in argv:
            return subprocess.CompletedProcess(argv, 0, COMPOSE_PS, "")
        if "/get_recording_status" in joined:
            return subprocess.CompletedProcess(argv, 0, RECORDING_RESPONSE, "")
        if "/get_topics" in joined:
            return subprocess.CompletedProcess(argv, 0, TOPICS_RESPONSE, "")
        if "/hardware_state_controller/set_state" in joined:
            return subprocess.CompletedProcess(argv, 0, "response:\nAccepted(ok=True)\n", "")
        return subprocess.CompletedProcess(argv, 0, "", "")

    monkeypatch.setattr(subprocess, "run", fake_run)
    robot.launched = []
    monkeypatch.setattr(
        subprocess, "Popen", lambda argv, **kwargs: robot.launched.append(argv) or None
    )
    return robot


# --- identity ----------------------------------------------------------------


def test_the_adapter_names_the_card_kind():
    assert AnvilAdapter.kind == KIND == "anvil-openarm-v2"


# --- status ------------------------------------------------------------------


def test_status_reports_the_contract_fields(adapter):
    reported = adapter.status()
    assert set(reported) >= {"mode", "hardware", "recording", "services", "disk"}
    assert reported["mode"] == "openarm_v2_leader_only.yaml"
    assert reported["hardware"] == "running"
    assert reported["recording"] is False
    assert reported["services"] == [{"name": "ros2", "state": "running", "status": "Up 2 hours"}]


def test_recording_is_unknown_when_the_stack_is_down(adapter, monkeypatch):
    """A down stack answers no service call; that is not the same as `not recording`."""
    monkeypatch.setattr(
        subprocess, "run", lambda argv, **kw: subprocess.CompletedProcess(argv, 1, "", "no service")
    )
    assert adapter.status()["recording"] is None


# --- mode --------------------------------------------------------------------


def test_the_mode_list_comes_from_the_real_config_directory(adapter):
    assert adapter.list_configs() == [
        "openarm_v2_leader_only.yaml",
        "openarm_v2_quest_teleop.yaml",
    ]


def test_a_mode_not_in_the_config_directory_is_refused(adapter):
    outcome = adapter.set_mode("openarm_v2_quest_teleop.yaml.evil")
    assert outcome.ok is False
    assert adapter.read_mode() == "openarm_v2_leader_only.yaml"
    assert adapter.launched == []


def test_setting_a_mode_rewrites_the_line_and_recreates_the_stack(adapter, loader):
    outcome = adapter.set_mode("openarm_v2_quest_teleop.yaml")
    assert outcome.ok is True
    assert adapter.read_mode() == "openarm_v2_quest_teleop.yaml"
    assert adapter.launched == [["docker", "compose", "up", "-d", "--force-recreate"]]
    written = loader.joinpath(".env.config").read_text(encoding="utf-8")
    assert "ROS_DOMAIN_ID=1" in written
    assert "ENABLE_CYCLONEDDS=true" in written


def test_setting_a_mode_twice_leaves_one_assignment(adapter, loader):
    adapter.set_mode("openarm_v2_quest_teleop.yaml")
    adapter.set_mode("openarm_v2_leader_only.yaml")
    written = loader.joinpath(".env.config").read_text(encoding="utf-8")
    assert written.count(MODE_KEY + "=") == 1


# --- stack -------------------------------------------------------------------


def test_up_and_down_detach(adapter):
    adapter.up()
    adapter.down()
    assert adapter.launched == [
        ["docker", "compose", "up", "-d"],
        ["docker", "compose", "down"],
    ]


# --- stop --------------------------------------------------------------------


def test_stop_pauses_the_hardware_state_controller(adapter, monkeypatch):
    seen = []
    monkeypatch.setattr(
        subprocess,
        "run",
        lambda argv, **kw: seen.append(argv) or subprocess.CompletedProcess(argv, 0, "", ""),
    )
    outcome = adapter.stop()
    assert outcome.detail["state"] == "pause"
    assert seen[0][-3:] == [
        "/hardware_state_controller/set_state",
        "anvil_msgs/srv/SetHardwareState",
        "{state: pause}",
    ]


def test_stop_never_asks_for_estop(adapter, monkeypatch):
    """No e-stop exists on this robot; the fleet must not pretend one does."""
    seen = []
    monkeypatch.setattr(
        subprocess,
        "run",
        lambda argv, **kw: seen.append(" ".join(argv)) or subprocess.CompletedProcess(argv, 0, "", ""),
    )
    adapter.stop()
    assert not any("estop" in call for call in seen)


def test_a_failed_service_call_surfaces_the_reason(adapter, monkeypatch):
    monkeypatch.setattr(
        subprocess,
        "run",
        lambda argv, **kw: subprocess.CompletedProcess(argv, 1, "", "service unavailable"),
    )
    with pytest.raises(AdapterError, match="service unavailable"):
        adapter.stop()


# --- recording ---------------------------------------------------------------


def test_record_start_reuses_an_existing_session(adapter):
    outcome = adapter.record("grocery-sort-v1", "start")
    assert outcome.ok is True
    assert outcome.detail["episode"] == "42"
    assert ("mutation", "recording.start", {"sessionId": 7}) in adapter.webapp.calls


def test_record_start_creates_a_session_with_the_rigs_own_topics(adapter):
    adapter.record("sixty-60-demo", "start")
    created = next(c for c in adapter.webapp.calls if c[1] == "recording.sessions.create")
    assert created[2]["slug"] == "sixty-60-demo"
    assert created[2]["topics"] == ["/joint_states", "/cam_wrist_l/image_raw"]


def test_record_stop_goes_through_the_webapp(adapter):
    """Never through the ROS graph: the webapp kills a recording it did not start."""
    adapter.record("grocery-sort-v1", "stop")
    assert any(call[1] == "recording.stop" for call in adapter.webapp.calls)


# --- episodes ----------------------------------------------------------------


def test_episodes_lists_recorded_directories(adapter):
    listed = adapter.episodes("grocery-sort-v1")
    assert [e["slug"] for e in listed] == ["0000"]
    assert listed[0]["metadata_yaml"] == "task: pick\n"


def test_an_unknown_dataset_lists_nothing(adapter):
    assert adapter.episodes("no-such-dataset") == []


def test_a_directory_that_is_not_an_episode_is_skipped(adapter, loader):
    (loader / "data" / "recordings" / "grocery-sort-v1" / "notes").mkdir()
    assert [e["slug"] for e in adapter.episodes("grocery-sort-v1")] == ["0000"]


def test_an_oversized_metadata_file_is_truncated(adapter, loader):
    """The directory is operator-writable; one huge file must not exhaust the agent."""
    from fm_robot_agent.anvil import METADATA_MAX_BYTES

    path = loader / "data" / "recordings" / "grocery-sort-v1" / "0000" / "metadata.yaml"
    path.write_text("x" * (METADATA_MAX_BYTES + 4096), encoding="utf-8")
    assert len(adapter.episodes("grocery-sort-v1")[0]["metadata_yaml"]) == METADATA_MAX_BYTES
