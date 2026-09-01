"""The Anvil adapter, against a compose stub and a webapp stub.

Both mocks sit at a system boundary — the ``docker`` subprocess and the webapp's
socket — so what the tests exercise is the adapter's own behaviour and nothing
about how it is wired together.
"""

from __future__ import annotations

import json
import os
import shlex
import subprocess

import pytest

from fm_robot_agent import anvil
from fm_robot_agent.anvil import KIND, MODE_KEY, AnvilAdapter
from fm_robot_agent.config import MODE, MOTION, SEVERING, TUNING, Journal
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
        "# the vendor's own comment, which a write must not eat\n"
        f"ROS_DOMAIN_ID=1\n{MODE_KEY}=openarm_v2_leader_only.yaml\nENABLE_CYCLONEDDS=true\n"
        "CYCLONEDDS_IFACE=wlp8s0\nCYCLONEDDS_VERBOSITY=warning\n"
        "TELEOP_POSITION_SCALE=1.0\nANVIL_SOMETHING_NEW=whatever\n",
        encoding="utf-8",
    )
    (tmp_path / "fm-comms.env").write_text(
        "FM_ROUTER_ENDPOINT=tcp/router:7447\nFM_ROS_DOMAIN_ID=0\nROS_DOMAIN_ID=0\n",
        encoding="utf-8",
    )
    return tmp_path


@pytest.fixture
def adapter(loader, monkeypatch):
    robot = AnvilAdapter(
        loader_dir=loader,
        comms_env=loader / "fm-comms.env",
        journal=Journal(loader / "config-journal.json"),
    )
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
    called = seen[0][-1]
    assert "ros2 service call /hardware_state_controller/set_state" in called
    assert "anvil_msgs/srv/SetHardwareState" in called
    assert "{state: pause}" in called


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


# --- what a service call has to set up ---------------------------------------

#: A path that would run a second command if it reached a shell unquoted.
HOSTILE_PATH = "/ws/setup.bash; id; echo '"


def _service_command(adapter, monkeypatch) -> str:
    seen = []
    monkeypatch.setattr(
        subprocess,
        "run",
        lambda argv, **kw: seen.append(argv) or subprocess.CompletedProcess(argv, 0, "", ""),
    )
    adapter.stop()
    return seen[0][-1]


def test_a_service_call_sources_what_the_entrypoint_would_have(adapter, monkeypatch):
    """`docker compose exec` skips the entrypoint, so `ros2` is not on PATH."""
    assert "source /opt/ros/$ROS_DISTRO/setup.bash" in _service_command(adapter, monkeypatch)


def test_a_service_call_sources_the_workspace_overlay(adapter, monkeypatch):
    """anvil_msgs is built into the overlay, not the base install."""
    assert "source /workspace/install/setup.bash" in _service_command(adapter, monkeypatch)


def test_a_service_call_names_the_rmw_the_image_ships(adapter, monkeypatch):
    """Fast DDS reaches the service and then fails decoding its reply."""
    assert "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in _service_command(adapter, monkeypatch)


def test_a_service_call_runs_in_one_shell(adapter, monkeypatch):
    """Sourcing and calling have to share a shell, or the sourcing buys nothing."""
    seen = []
    monkeypatch.setattr(
        subprocess,
        "run",
        lambda argv, **kw: seen.append(argv) or subprocess.CompletedProcess(argv, 0, "", ""),
    )
    adapter.stop()
    assert seen[0][:7] == ["docker", "compose", "exec", "-T", "ros2", "bash", "-c"]


def test_an_env_supplied_path_cannot_extend_the_command(adapter, monkeypatch):
    """Config is closer to home than the fabric, and still not a place to trust."""
    monkeypatch.setattr("fm_robot_agent.anvil.WORKSPACE_SETUP", HOSTILE_PATH)
    called = _service_command(adapter, monkeypatch)
    assert f"source {shlex.quote(HOSTILE_PATH)} &&" in called


def test_an_env_supplied_rmw_cannot_extend_the_command(adapter, monkeypatch):
    monkeypatch.setattr("fm_robot_agent.anvil.RMW", "rmw_x; id")
    called = _service_command(adapter, monkeypatch)
    assert "RMW_IMPLEMENTATION='rmw_x; id'" in called


# --- config ------------------------------------------------------------------


def settings(adapter) -> dict:
    return {setting.key: setting for setting in adapter.config_read()}


def test_config_read_comes_from_the_robots_own_file(adapter):
    """Anvil version-controls `.env.config`; a key list held here would go stale."""
    listed = settings(adapter)
    assert listed["ROS_DOMAIN_ID"].klass == SEVERING
    assert listed["TELEOP_POSITION_SCALE"].klass == MOTION
    assert listed["CYCLONEDDS_VERBOSITY"].klass == TUNING
    assert listed[MODE_KEY].klass == MODE


def test_the_mode_keys_options_are_the_configs_on_disk(adapter):
    assert settings(adapter)[MODE_KEY].options == (
        "openarm_v2_leader_only.yaml",
        "openarm_v2_quest_teleop.yaml",
    )


def test_an_unclassified_key_is_listed_and_refused(adapter, loader):
    """`.env.config` is an env_file for a privileged container."""
    assert settings(adapter)["ANVIL_SOMETHING_NEW"].writable is False
    outcome = adapter.config_write("ANVIL_SOMETHING_NEW", "anything")
    assert outcome.ok is False
    assert "whatever" in (loader / ".env.config").read_text()


def test_a_tuning_key_is_written_without_recreating_the_stack(adapter, loader):
    outcome = adapter.config_write("CYCLONEDDS_VERBOSITY", "fine")
    assert outcome.ok is True
    assert "CYCLONEDDS_VERBOSITY=fine" in (loader / ".env.config").read_text()
    assert adapter.launched == []


def test_a_write_keeps_every_other_line(adapter, loader):
    adapter.config_write("CYCLONEDDS_VERBOSITY", "fine")
    written = (loader / ".env.config").read_text()
    assert "# the vendor's own comment, which a write must not eat" in written
    assert "ENABLE_CYCLONEDDS=true" in written


@pytest.mark.parametrize(
    ("key", "value"),
    [
        ("ROS_DOMAIN_ID", "999"),
        ("ROS_DOMAIN_ID", "one"),
        ("CYCLONEDDS_IFACE", "not an interface"),
        ("CYCLONEDDS_VERBOSITY", "loud"),
        ("CYCLONEDDS_TRANSPORT", "carrier-pigeon"),
        ("ENABLE_CYCLONEDDS", "yes"),
        ("CYCLONEDDS_PEER_IP", "300.1.1.1"),
        ("TELEOP_POSITION_SCALE", "-1"),
        (MODE_KEY, "openarm_v2_quest_teleop.yaml.evil"),
    ],
)
def test_a_value_the_key_cannot_hold_is_refused(adapter, loader, key, value):
    before = (loader / ".env.config").read_text()
    assert adapter.config_write(key, value).ok is False
    assert (loader / ".env.config").read_text() == before


def test_a_motion_key_is_refused_while_the_stack_is_up(adapter, loader):
    outcome = adapter.config_write("TELEOP_POSITION_SCALE", "1.5")
    assert outcome.ok is False
    assert "TELEOP_POSITION_SCALE=1.0" in (loader / ".env.config").read_text()


def test_a_motion_key_is_written_once_the_stack_is_down(adapter, loader, monkeypatch):
    monkeypatch.setattr(adapter, "compose_ps", list)
    assert adapter.config_write("TELEOP_POSITION_SCALE", "1.5").ok is True
    assert "TELEOP_POSITION_SCALE=1.5" in (loader / ".env.config").read_text()


def test_a_severing_key_recreates_the_stack(adapter):
    assert adapter.config_write("CYCLONEDDS_IFACE", "eth0").ok is True
    assert adapter.launched and "--force-recreate" in adapter.launched[0]


# --- the paired write --------------------------------------------------------


def test_the_domain_lands_in_both_files(adapter, loader):
    """A bridge left on the fleet's domain while the stack moved is defect #2."""
    assert adapter.config_write("ROS_DOMAIN_ID", "7").ok is True
    comms = (loader / "fm-comms.env").read_text()
    assert "FM_ROS_DOMAIN_ID=7" in comms
    assert "ROS_DOMAIN_ID=7" in comms
    assert "ROS_DOMAIN_ID=7" in (loader / ".env.config").read_text()


def test_the_interface_lands_in_both_files(adapter, loader):
    assert adapter.config_write("CYCLONEDDS_IFACE", "eth0").ok is True
    assert "FM_DDS_IFACE=eth0" in (loader / "fm-comms.env").read_text()


def test_an_unpaired_key_leaves_the_fleet_file_alone(adapter, loader):
    before = (loader / "fm-comms.env").read_text()
    adapter.config_write("CYCLONEDDS_VERBOSITY", "fine")
    assert (loader / "fm-comms.env").read_text() == before


def test_a_write_that_fails_on_the_second_file_leaves_the_first_unchanged(
    adapter, loader, monkeypatch
):
    """Half a paired write is exactly the disagreement the pairing exists to stop."""
    before = (loader / ".env.config").read_text()
    real_replace = os.replace

    def fail_on_the_fleet_file(src, dst):
        if str(dst).endswith("fm-comms.env"):
            raise OSError("read-only file system")
        return real_replace(src, dst)

    monkeypatch.setattr(os, "replace", fail_on_the_fleet_file)
    with pytest.raises(AdapterError):
        adapter.config_write("ROS_DOMAIN_ID", "7")
    monkeypatch.undo()
    assert (loader / ".env.config").read_text() == before


def test_a_host_with_no_fleet_file_still_writes_its_own(adapter, loader):
    (loader / "fm-comms.env").unlink()
    assert adapter.config_write("ROS_DOMAIN_ID", "7").ok is True
    assert not (loader / "fm-comms.env").exists()


# --- the severing guard ------------------------------------------------------


@pytest.fixture
def dead_telemetry(adapter, monkeypatch):
    """A robot whose data plane never comes back, verified instantly."""
    monkeypatch.setattr(adapter, "_telemetry_arrives", lambda: False)
    monkeypatch.setattr(anvil, "VERIFY_WINDOW_S", 0.0)
    monkeypatch.setattr(anvil, "VERIFY_INTERVAL_S", 0.0)
    return adapter


def test_a_severing_write_that_keeps_telemetry_is_kept(adapter, loader):
    outcome = adapter.config_write("CYCLONEDDS_IFACE", "eth0")
    assert outcome.ok is True
    assert outcome.detail["verified"] is True
    assert "CYCLONEDDS_IFACE=eth0" in (loader / ".env.config").read_text()
    assert adapter.journal.read() is None


def test_a_severing_write_that_kills_telemetry_reverts_both_files(dead_telemetry, loader):
    """The three silent defects of 2026-09-01, made undoable on demand."""
    before_config = (loader / ".env.config").read_text()
    before_comms = (loader / "fm-comms.env").read_text()
    outcome = dead_telemetry.config_write("ROS_DOMAIN_ID", "7")
    assert outcome.ok is False
    assert outcome.detail["verified"] is False
    assert (loader / ".env.config").read_text() == before_config
    assert (loader / "fm-comms.env").read_text() == before_comms
    assert dead_telemetry.journal.read() is None


def test_a_revert_restarts_the_stack_and_the_bridge(dead_telemetry):
    dead_telemetry.config_write("ROS_DOMAIN_ID", "7")
    # Once to apply the change, once to put the old one back.
    assert len(dead_telemetry.launched) == 2


def test_a_second_severing_write_while_one_is_open_is_refused(adapter, monkeypatch):
    """Two overlapping reverts would restore each other's intermediate state."""
    monkeypatch.setattr(adapter, "_data_plane_returns", lambda: False)
    monkeypatch.setattr(adapter, "_revert", lambda previous: [])
    adapter.config_write("ROS_DOMAIN_ID", "7")
    assert adapter.config_write("CYCLONEDDS_IFACE", "eth0").ok is False


def test_an_agent_that_starts_with_an_open_window_finishes_it(adapter, loader):
    """The journal closes only after telemetry is seen; an open one outlived its agent."""
    before = (loader / ".env.config").read_text()
    adapter.journal.open("ROS_DOMAIN_ID", "7", {loader / ".env.config": before})
    (loader / ".env.config").write_text("ROS_DOMAIN_ID=7\n", encoding="utf-8")

    outcome = adapter.finish_open_change()

    assert outcome.ok is True
    assert (loader / ".env.config").read_text() == before
    assert adapter.journal.read() is None


def test_an_agent_that_starts_with_no_open_window_does_nothing(adapter):
    assert adapter.finish_open_change() is None


def test_rollback_with_nothing_open_says_so(adapter):
    assert adapter.config_rollback().ok is False
