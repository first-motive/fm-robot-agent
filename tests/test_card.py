"""The identity card reader. Every path uses a card written into tmp_path."""

from __future__ import annotations

import json

import pytest

from fm_robot_agent.card import SCHEMA_VERSION, CardError, RobotCard, read_card

VALID = {
    "schema_version": SCHEMA_VERSION,
    "name": "fm-rob-01",
    "role": "robot",
    "robot": "anvil-openarm-v2",
    "fleet": "first-motive",
    "transport": "zenoh",
    "workspace": "/home/anvil",
}


def write_card(tmp_path, **overrides):
    card = {**VALID, **overrides}
    for key, value in list(card.items()):
        if value is None:
            del card[key]
    path = tmp_path / "machine.json"
    path.write_text(json.dumps(card), encoding="utf-8")
    return path


def test_a_robot_card_reads(tmp_path):
    card = read_card(write_card(tmp_path))
    assert card == RobotCard(name="fm-rob-01", kind="anvil-openarm-v2")


def test_the_namespace_is_derived_from_the_name():
    """Never stored: a namespace written twice drifts the moment a robot is renamed."""
    assert RobotCard(name="fm-rob-01", kind="axol").namespace == "fm_rob_01"


def test_an_absent_card_is_refused(tmp_path):
    with pytest.raises(CardError, match="does not exist"):
        read_card(tmp_path / "nothing.json")


def test_an_unreadable_card_is_refused(tmp_path):
    path = tmp_path / "machine.json"
    path.write_text("{not json", encoding="utf-8")
    with pytest.raises(CardError, match="not readable as JSON"):
        read_card(path)


def test_an_unknown_schema_is_refused_rather_than_guessed(tmp_path):
    with pytest.raises(CardError, match="schema_version"):
        read_card(write_card(tmp_path, schema_version=SCHEMA_VERSION + 1))


def test_a_card_that_is_not_a_robots_is_refused(tmp_path):
    with pytest.raises(CardError, match="role"):
        read_card(write_card(tmp_path, role="jetson"))


def test_an_unknown_robot_kind_is_refused(tmp_path):
    """No adapter exists for it, so serving its verbs would answer nothing."""
    with pytest.raises(CardError, match="robot"):
        read_card(write_card(tmp_path, robot="humanoid"))


def test_a_nameless_card_is_refused(tmp_path):
    with pytest.raises(CardError, match="no name"):
        read_card(write_card(tmp_path, name=None))
