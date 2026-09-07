"""The `fm robot` client: argument handling and reply rendering, with no session."""

from __future__ import annotations

import argparse
import json

import pytest

from fm_robot_agent import client


def args(**overrides) -> argparse.Namespace:
    return argparse.Namespace(**{"value": None, "dataset": "", "arg": None, "task": "", **overrides})


def test_a_device_name_derives_the_namespace():
    assert client.namespace_of("fm-rob-01") == "fm_rob_01"


@pytest.mark.parametrize("verb", ["status", "up", "down", "stop", "episodes"])
def test_verbs_without_a_body_send_none(verb):
    assert client.build_payload(verb, args()) is None


def test_mode_is_a_verb_of_its_own():
    """A mode's arguments are not configuration, so it stopped being a config write.

    Almond's `run-policy` needs a policy, its type and a task per start, and the
    robot stores none of them. Sending those through a settings write would have
    made the wire describe the wrong thing.
    """
    assert client.build_payload("mode", args(value="openarm_v2_quest_teleop.yaml")) == {
        "value": "openarm_v2_quest_teleop.yaml",
    }
    assert client.wire_verb("mode") == "mode"


def test_mode_carries_the_arguments_an_operation_needs():
    payload = client.build_payload(
        "mode",
        args(
            value="policy",
            arg=["policy_path=first-motive/policy-act-1", "policy_type=act", "task=open the cup"],
        ),
    )
    assert payload == {
        "value": "policy",
        "args": {
            "policy_path": "first-motive/policy-act-1",
            "policy_type": "act",
            "task": "open the cup",
        },
    }


def test_record_defaults_to_starting():
    assert client.build_payload("record", args(dataset="pick-place")) == {
        "dataset": "pick-place",
        "action": "start",
    }


def test_record_stop_is_explicit():
    assert client.build_payload("record", args(dataset="pick-place", value="stop"))["action"] == "stop"


def test_record_carries_the_task_when_one_is_given():
    """The instruction a language-conditioned policy trains on, onto the wire."""
    payload = client.build_payload(
        "record", args(dataset="checkers-bag-v1", task="put the nuts in the bag")
    )
    assert payload == {
        "dataset": "checkers-bag-v1",
        "action": "start",
        "task": "put the nuts in the bag",
    }


def test_record_without_a_task_sends_no_task_key():
    """An absent instruction is absent, not an empty sentence the robot has to read."""
    assert "task" not in client.build_payload("record", args(dataset="pick-place"))


# --- usage -------------------------------------------------------------------


@pytest.mark.parametrize(
    "argv",
    [
        ["fm-rob-01"],                      # no verb
        ["fm-rob-01", "mode"],              # mode without a config
        ["fm-rob-01", "record", "start"],   # record without a dataset
        ["fm-rob-01", "episodes"],          # episodes without a dataset
        ["fm-rob-01", "reboot"],            # a verb the contract has no key for
        ["fm-rob-01", "config", "reboot"],  # an action config does not have
        ["fm-rob-01", "config", "set"],     # set without KEY=VALUE
        ["fm-rob-01", "config", "set", "CYCLONEDDS_VERBOSITY"],  # no value
    ],
)
def test_incomplete_invocations_are_refused_before_any_session_opens(argv):
    with pytest.raises(SystemExit) as exit_info:
        client.main(argv)
    assert exit_info.value.code == client.EX_USAGE


@pytest.mark.parametrize(
    "device",
    ["fm-rob-*", "*", "fm-rob-01/status", "fm/robot/fm_rob_02/stop", "fm-rob-**", "rob-01"],
)
def test_a_device_name_that_is_a_key_expression_is_refused(device):
    """`*` and `/` are syntax in a key, so a name carrying either would widen the query."""
    with pytest.raises(SystemExit) as exit_info:
        client.main([device, "status"])
    assert exit_info.value.code == client.EX_USAGE


def test_a_reply_cannot_repaint_the_terminal(capsys):
    """Reply text comes off the fabric; an escape sequence in it is not output."""
    client.render([{"ok": False, "error": "\x1b[2Kdone\rrefused: nothing"}], as_json=False)
    printed = capsys.readouterr().out
    assert "\x1b" not in printed
    assert "\r" not in printed


def test_no_router_endpoint_is_a_precondition_failure(monkeypatch, tmp_path, capsys):
    monkeypatch.delenv("FM_ROUTER_ENDPOINT", raising=False)
    monkeypatch.setenv("FM_COMMS_ENV_FILE", str(tmp_path / "absent.env"))
    assert client.main(["fm-rob-01", "status"]) == client.EX_PRECONDITION
    assert "FM_ROUTER_ENDPOINT" in capsys.readouterr().err


# --- rendering ---------------------------------------------------------------


def test_json_output_unwraps_a_single_reply(capsys):
    client.render([{"schema_version": 1, "ok": True, "mode": "teleop"}], as_json=True)
    assert json.loads(capsys.readouterr().out)["mode"] == "teleop"


def test_json_output_keeps_a_list_when_many_robots_answer(capsys):
    client.render([{"ok": True, "kind": "axol"}, {"ok": True, "kind": "anvil-openarm-v2"}], as_json=True)
    assert len(json.loads(capsys.readouterr().out)) == 2


def test_a_refusal_is_printed_with_its_reason(capsys):
    client.render([{"ok": False, "error": "unknown config"}], as_json=False)
    assert "refused: unknown config" in capsys.readouterr().out


def test_the_schema_stamp_is_not_shown_to_a_person(capsys):
    client.render([{"schema_version": 1, "ok": True, "mode": "teleop"}], as_json=False)
    assert "schema_version" not in capsys.readouterr().out


# --- config ------------------------------------------------------------------


def test_config_defaults_to_reading():
    assert client.build_payload("config", args()) == {"action": "get"}


def test_config_set_splits_the_assignment():
    assert client.build_payload("config", args(value="set", assignment="ROS_DOMAIN_ID=7")) == {
        "action": "set",
        "key": "ROS_DOMAIN_ID",
        "value": "7",
    }


def test_config_rollback_carries_nothing_else():
    assert client.build_payload("config", args(value="rollback")) == {"action": "rollback"}


def test_config_get_prints_a_line_per_key_with_its_class(capsys):
    client.render(
        [
            {
                "schema_version": 1,
                "ok": True,
                "config": [
                    {"key": "ROS_DOMAIN_ID", "value": "1", "class": "severing", "writable": True},
                    {"key": "ANVIL_NEW", "value": "x", "class": "unknown", "writable": False},
                ],
            }
        ],
        as_json=False,
    )
    printed = capsys.readouterr().out.splitlines()
    assert printed[0] == "ROS_DOMAIN_ID=1  [severing]"
    assert printed[1] == "ANVIL_NEW=x  [unknown]  read-only"


def test_a_refusal_names_the_guard_that_refused_it(capsys):
    client.render(
        [{"ok": False, "message": "the stack is up", "class": "motion"}],
        as_json=False,
    )
    assert capsys.readouterr().out.strip() == "refused (motion): the stack is up"


def test_a_config_write_waits_longer_than_any_other_verb():
    """A severing write is answered only once telemetry has come back."""
    assert client.CONFIG_TIMEOUT_S > client.QUERY_TIMEOUT_S
