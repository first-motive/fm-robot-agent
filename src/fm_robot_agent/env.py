"""Where this host finds the router.

``/etc/fm-comms.env`` holds what the whole fleet shares — the router endpoint,
its port, the ROS domain — and fm-comms' own units read it as their
``EnvironmentFile``. This agent's unit does the same, so on a provisioned host
the value is already in the environment and nothing here parses anything.

The file is read only as the fallback, which is what makes ``uv run
fm-robot-agent`` on a robot land on the same router the unit would use instead of
failing for a reason the operator has to go looking for.
"""

from __future__ import annotations

import os
from pathlib import Path

ROUTER_ENDPOINT_KEY = "FM_ROUTER_ENDPOINT"
COMMS_ENV_FILE = "FM_COMMS_ENV_FILE"
DEFAULT_COMMS_ENV = Path("/etc/fm-comms.env")


class EndpointError(Exception):
    """This host cannot say where its router is."""


def comms_env_path() -> Path:
    """The fleet-wide env file, honouring the same override fm-comms uses."""
    override = os.environ.get(COMMS_ENV_FILE, "").strip()
    return Path(override).expanduser() if override else DEFAULT_COMMS_ENV


def read_env_file(path: Path) -> dict[str, str]:
    """Parse a shell-style env file into a dict, ignoring comments and blanks.

    Deliberately not a shell: the file holds plain ``KEY=value`` assignments, and
    sourcing it would run whatever else it contained. Later assignments win, as
    they would in a shell.
    """
    values: dict[str, str] = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return values
    for line in lines:
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or "=" not in stripped:
            continue
        key, _, value = stripped.partition("=")
        values[key.strip()] = value.strip().strip('"').strip("'")
    return values


def router_endpoint() -> str:
    """The router this host connects to, or raise :class:`EndpointError`."""
    from_environ = os.environ.get(ROUTER_ENDPOINT_KEY, "").strip()
    if from_environ:
        return from_environ
    path = comms_env_path()
    endpoint = read_env_file(path).get(ROUTER_ENDPOINT_KEY, "").strip()
    if not endpoint:
        raise EndpointError(f"{ROUTER_ENDPOINT_KEY} is set neither in the environment nor in {path}")
    return endpoint
