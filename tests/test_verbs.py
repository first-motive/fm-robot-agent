"""The verb router, driven against the fake robot. No router, no hardware."""

from __future__ import annotations

import json

import pytest

from fm_robot_agent.config import MODE_ALIAS, MOTION, SEVERING, TUNING
from fm_robot_agent.fake import FakeAdapter
from fm_robot_agent.protocol import SCHEMA_VERSION, AdapterError, Outcome
from fm_robot_agent.verbs import KEY_PREFIX, answer, verb_of

NS = "fm_rob_01"


def key(verb: str, namespace: str = NS) -> str:
    return f"{KEY_PREFIX}/{namespace}/{verb}"


def body(reply) -> dict:
    return json.loads(reply.payload)


@pytest.fixture
def robot() -> FakeAdapter:
    return FakeAdapter()


def ask(robot, verb, *, parameters="", payload=None):
    encoded = json.dumps(payload).encode() if payload is not None else None
    return answer(key(verb), robot, NS, parameters=parameters, payload=encoded)


# --- key routing -------------------------------------------------------------


@pytest.mark.parametrize(
    "verb", ["status", "up", "down", "config", "record", "stop", "episodes"]
)
def test_every_contract_verb_routes(verb):
    assert verb_of(key(verb), NS) == verb


def test_another_robots_namespace_is_not_ours():
    """One agent serves one robot; answering for another would race its agent."""
    assert verb_of(key("status", "fm_rob_02"), NS) is None


def test_unknown_verb_is_refused(robot):
    reply = answer(key("reboot"), robot, NS)
    assert reply.ok is False
    assert "unknown key" in body(reply)["error"]


# --- envelope ----------------------------------------------------------------


def test_every_reply_carries_the_schema_version(robot):
    for verb in ("status", "up", "down", "stop"):
        assert body(ask(robot, verb))["schema_version"] == SCHEMA_VERSION


# --- read verbs --------------------------------------------------------------


def test_status_reports_the_kind_and_the_contract_fields(robot):
    reported = body(ask(robot, "status"))
    assert reported["kind"] == "fake"
    assert set(reported) >= {"mode", "hardware", "recording", "services", "disk"}


def test_episodes_needs_a_dataset(robot):
    assert ask(robot, "episodes").ok is False


def test_episodes_refuses_a_dataset_that_is_a_path(robot):
    reply = ask(robot, "episodes", parameters="dataset=../../etc")
    assert reply.ok is False
    assert body(reply)["error"] == "invalid dataset name"


def test_episodes_lists_what_was_recorded(robot):
    ask(robot, "record", payload={"dataset": "pick-place", "action": "start"})
    reply = ask(robot, "episodes", parameters="dataset=pick-place")
    assert [e["slug"] for e in body(reply)["episodes"]] == ["0000"]


# --- write verbs -------------------------------------------------------------


def test_up_and_down_move_the_hardware_state(robot):
    assert body(ask(robot, "up"))["ok"] is True
    assert body(ask(robot, "status"))["hardware"] == "running"
    ask(robot, "down")
    assert body(ask(robot, "status"))["hardware"] == "down"


# --- config ------------------------------------------------------------------


def config(robot, **fields):
    return body(ask(robot, "config", payload=fields))


def test_config_needs_a_known_action(robot):
    assert ask(robot, "config", payload={"action": "reboot"}).ok is False
    assert ask(robot, "config", payload={}).ok is False


def test_config_get_lists_every_key_with_its_class(robot):
    listed = {entry["key"]: entry for entry in config(robot, action="get")["config"]}
    assert listed["CYCLONEDDS_IFACE"]["class"] == SEVERING
    assert listed["TELEOP_POSITION_SCALE"]["class"] == MOTION
    assert listed["CYCLONEDDS_VERBOSITY"]["class"] == TUNING


def test_config_get_lists_an_unknown_key_and_set_refuses_it(robot):
    """`.env.config` feeds a privileged container; an unclassified write is injection."""
    listed = {entry["key"]: entry for entry in config(robot, action="get")["config"]}
    assert listed["ANVIL_SOMETHING_NEW"]["writable"] is False
    refused = config(robot, action="set", key="ANVIL_SOMETHING_NEW", value="anything")
    assert refused["ok"] is False


def test_config_set_needs_a_key_and_a_value(robot):
    assert ask(robot, "config", payload={"action": "set", "value": "x"}).ok is False
    assert ask(robot, "config", payload={"action": "set", "key": "CYCLONEDDS_VERBOSITY"}).ok is False


def test_config_set_accepts_a_number_the_caller_typed_as_json(robot):
    assert config(robot, action="set", key="TELEOP_POSITION_SCALE", value=1.5)["ok"] is True


def test_config_set_refuses_an_oversized_value(robot):
    from fm_robot_agent.verbs import MAX_VALUE_LEN

    long_value = "x" * (MAX_VALUE_LEN + 1)
    assert ask(
        robot, "config", payload={"action": "set", "key": "CYCLONEDDS_IFACE", "value": long_value}
    ).ok is False


def test_config_set_writes_a_tuning_key_through(robot):
    assert config(robot, action="set", key="CYCLONEDDS_VERBOSITY", value="fine")["ok"] is True
    assert robot.config["CYCLONEDDS_VERBOSITY"] == "fine"


def test_a_motion_key_is_refused_while_the_robot_is_busy(robot):
    ask(robot, "up")
    assert config(robot, action="set", key="TELEOP_POSITION_SCALE", value="1.5")["ok"] is False
    ask(robot, "down")
    assert config(robot, action="set", key="TELEOP_POSITION_SCALE", value="1.5")["ok"] is True


def test_the_mode_alias_reaches_the_robots_own_mode_key(robot):
    """`fm robot X mode <value>` is a config write; the adapter knows which key."""
    assert config(robot, action="set", key=MODE_ALIAS, value="teleop")["ok"] is True
    assert body(ask(robot, "status"))["mode"] == "teleop"
    assert robot.config["ARMS_CONTROL_CONFIG_FILE"] == "teleop"


def test_a_mode_the_robot_rejects_answers_rather_than_raises(robot):
    """A refusal is an ordinary reply — the caller needs the reason, not a timeout."""
    reply = ask(robot, "config", payload={"action": "set", "key": MODE_ALIAS, "value": "nonsense"})
    assert reply.ok is True
    assert body(reply)["ok"] is False


def test_rollback_with_nothing_open_says_so(robot):
    assert config(robot, action="rollback")["ok"] is False


def test_rollback_restores_what_a_severing_write_replaced(robot):
    config(robot, action="set", key="CYCLONEDDS_IFACE", value="docker0")
    assert config(robot, action="rollback")["ok"] is True
    assert robot.config["CYCLONEDDS_IFACE"] == "eth0"


def test_record_start_returns_the_episode(robot):
    reply = body(ask(robot, "record", payload={"dataset": "pick-place", "action": "start"}))
    assert reply["ok"] is True
    assert reply["episode"] == "0000"


def test_record_needs_a_known_action(robot):
    reply = ask(robot, "record", payload={"dataset": "pick-place", "action": "pause"})
    assert reply.ok is False


def test_record_refuses_a_dataset_that_is_a_path(robot):
    reply = ask(robot, "record", payload={"dataset": "../etc", "action": "start"})
    assert reply.ok is False


def test_stop_clears_recording_and_mode(robot):
    ask(robot, "record", payload={"dataset": "pick-place", "action": "start"})
    ask(robot, "config", payload={"action": "set", "key": MODE_ALIAS, "value": "teleop"})
    assert body(ask(robot, "stop"))["state"] == "idle"
    reported = body(ask(robot, "status"))
    assert reported["recording"] is None
    assert reported["mode"] == "idle"


def test_a_malformed_body_is_treated_as_empty(robot):
    reply = answer(key("config"), robot, NS, payload=b"{not json")
    assert reply.ok is False


def test_an_oversized_body_is_refused_without_parsing(robot):
    """A caller on the fabric must not be able to spend the agent's memory."""
    from fm_robot_agent.verbs import MAX_BODY_BYTES

    oversized = b'{"action": "get", "key": "' + b"x" * MAX_BODY_BYTES + b'"}'
    assert answer(key("config"), robot, NS, payload=oversized).ok is False


# --- adapter failure ---------------------------------------------------------


class UnreachableRobot:
    kind = "fake"

    def status(self) -> dict:
        raise AdapterError("robot did not answer")

    def up(self) -> Outcome: ...
    def down(self) -> Outcome: ...
    def set_mode(self, config: str) -> Outcome: ...
    def config_read(self) -> list: ...
    def config_write(self, key: str, value: str) -> Outcome: ...
    def config_rollback(self) -> Outcome: ...
    def record(self, dataset: str, action: str) -> Outcome: ...
    def stop(self) -> Outcome: ...
    def episodes(self, dataset: str) -> list[dict]: ...


def test_an_unreachable_robot_replies_with_the_reason():
    reply = answer(key("status"), UnreachableRobot(), NS)
    assert reply.ok is False
    assert body(reply)["error"] == "robot did not answer"


# --- discovery ---------------------------------------------------------------


def test_a_discovery_query_is_answered():
    """`fm/robot/*/status` is how a caller finds robots it cannot name yet."""
    assert verb_of(f"{KEY_PREFIX}/*/status", NS) == "status"


def test_a_discovery_query_still_only_answers_known_verbs():
    assert verb_of(f"{KEY_PREFIX}/*/reboot", NS) is None


def test_another_robots_namespace_is_still_refused():
    assert verb_of(key("status", "fm_rob_02"), NS) is None


def test_a_partial_namespace_is_not_a_wildcard():
    """Only a bare `*` discovers; a glob against our name is somebody guessing."""
    assert verb_of(f"{KEY_PREFIX}/fm_rob_*/status", NS) is None


def test_a_key_with_extra_segments_is_refused():
    assert verb_of(f"{KEY_PREFIX}/{NS}/status/extra", NS) is None
    assert verb_of(f"other/robot/{NS}/status", NS) is None


def test_a_reply_names_the_robot_that_answered_not_the_query(robot):
    """A discovery reply echoing the selector back would identify nobody."""
    reply = answer(f"{KEY_PREFIX}/*/status", robot, NS)
    assert reply.key == f"{KEY_PREFIX}/{NS}/status"
    assert body(reply)["kind"] == "fake"


def test_a_named_query_still_replies_under_that_name(robot):
    assert answer(key("status"), robot, NS).key == f"{KEY_PREFIX}/{NS}/status"


@pytest.mark.parametrize("verb", ["up", "down", "config", "record", "stop"])
def test_a_wildcard_discovers_but_never_commands(verb):
    """`fm/robot/*/down` would otherwise take the whole fleet down in one query."""
    assert verb_of(f"{KEY_PREFIX}/*/{verb}", NS) is None


@pytest.mark.parametrize("verb", ["status", "episodes"])
def test_a_wildcard_still_reads(verb):
    assert verb_of(f"{KEY_PREFIX}/*/{verb}", NS) == verb


@pytest.mark.parametrize("verb", ["up", "down", "config", "record", "stop"])
def test_a_named_query_still_commands(verb):
    assert verb_of(key(verb), NS) == verb
