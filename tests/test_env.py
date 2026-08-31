"""Finding the router: the environment first, the fleet-wide file as fallback."""

from __future__ import annotations

import pytest

from fm_robot_agent.env import (
    COMMS_ENV_FILE,
    ROUTER_ENDPOINT_KEY,
    EndpointError,
    read_env_file,
    router_endpoint,
)

ENDPOINT = "tcp/rune.example.ts.net:7447"


def write_env(tmp_path, text):
    path = tmp_path / "fm-comms.env"
    path.write_text(text, encoding="utf-8")
    return path


def test_the_environment_wins(monkeypatch, tmp_path):
    """On a provisioned host systemd has already sourced the file; nothing parses."""
    monkeypatch.setenv(ROUTER_ENDPOINT_KEY, ENDPOINT)
    monkeypatch.setenv(COMMS_ENV_FILE, str(write_env(tmp_path, "FM_ROUTER_ENDPOINT=tcp/other:7447\n")))
    assert router_endpoint() == ENDPOINT


def test_the_file_is_the_fallback(monkeypatch, tmp_path):
    monkeypatch.delenv(ROUTER_ENDPOINT_KEY, raising=False)
    monkeypatch.setenv(COMMS_ENV_FILE, str(write_env(tmp_path, f"{ROUTER_ENDPOINT_KEY}={ENDPOINT}\n")))
    assert router_endpoint() == ENDPOINT


def test_no_endpoint_anywhere_names_both_places(monkeypatch, tmp_path):
    monkeypatch.delenv(ROUTER_ENDPOINT_KEY, raising=False)
    monkeypatch.setenv(COMMS_ENV_FILE, str(tmp_path / "absent.env"))
    with pytest.raises(EndpointError, match="absent.env"):
        router_endpoint()


def test_comments_blanks_and_quotes_are_handled(tmp_path):
    path = write_env(
        tmp_path,
        "# a comment\n\nFM_ROUTER_PORT=7447\n"
        f'{ROUTER_ENDPOINT_KEY}="{ENDPOINT}"\n'
        "  spaced = value \n",
    )
    values = read_env_file(path)
    assert values[ROUTER_ENDPOINT_KEY] == ENDPOINT
    assert values["FM_ROUTER_PORT"] == "7447"
    assert values["spaced"] == "value"


def test_the_last_assignment_wins(tmp_path):
    """As it would in a shell — the file is parsed, never sourced."""
    path = write_env(tmp_path, f"{ROUTER_ENDPOINT_KEY}=tcp/first:7447\n{ROUTER_ENDPOINT_KEY}={ENDPOINT}\n")
    assert read_env_file(path)[ROUTER_ENDPOINT_KEY] == ENDPOINT
