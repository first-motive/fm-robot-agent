"""The verb router, driven against the fake robot. No router, no hardware."""

from __future__ import annotations

import json

import pytest

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
    "verb", ["status", "up", "down", "mode", "record", "stop", "episodes"]
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


def test_mode_without_a_config_is_refused(robot):
    assert ask(robot, "mode", payload={}).ok is False


def test_mode_the_robot_rejects_answers_rather_than_raises(robot):
    """A refusal is an ordinary reply — the caller needs the reason, not a timeout."""
    reply = ask(robot, "mode", payload={"config": "nonsense"})
    assert reply.ok is True
    assert body(reply)["ok"] is False


def test_mode_switches(robot):
    assert body(ask(robot, "mode", payload={"config": "teleop"}))["ok"] is True
    assert body(ask(robot, "status"))["mode"] == "teleop"


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
    ask(robot, "mode", payload={"config": "teleop"})
    assert body(ask(robot, "stop"))["state"] == "idle"
    reported = body(ask(robot, "status"))
    assert reported["recording"] is None
    assert reported["mode"] == "idle"


def test_a_malformed_body_is_treated_as_empty(robot):
    reply = answer(key("mode"), robot, NS, payload=b"{not json")
    assert reply.ok is False


def test_an_oversized_body_is_refused_without_parsing(robot):
    """A caller on the fabric must not be able to spend the agent's memory."""
    from fm_robot_agent.verbs import MAX_BODY_BYTES

    oversized = b'{"config": "' + b"x" * MAX_BODY_BYTES + b'"}'
    assert answer(key("mode"), robot, NS, payload=oversized).ok is False


# --- adapter failure ---------------------------------------------------------


class UnreachableRobot:
    kind = "fake"

    def status(self) -> dict:
        raise AdapterError("robot did not answer")

    def up(self) -> Outcome: ...
    def down(self) -> Outcome: ...
    def set_mode(self, config: str) -> Outcome: ...
    def record(self, dataset: str, action: str) -> Outcome: ...
    def stop(self) -> Outcome: ...
    def episodes(self, dataset: str) -> list[dict]: ...


def test_an_unreachable_robot_replies_with_the_reason():
    reply = answer(key("status"), UnreachableRobot(), NS)
    assert reply.ok is False
    assert body(reply)["error"] == "robot did not answer"
