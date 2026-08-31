"""This host's identity card, as the agent needs it.

The card is the one file that says what a machine is. ``fm machine init`` in
fm-setup writes it; nothing here ever does. The agent reads three things from it:
the name (which derives the namespace every key sits under), the role (which must
be ``robot``), and the ``robot`` field naming which vendor stack this host runs.

Nothing is written down twice. The namespace is derived from the name rather than
stored, because a namespace recorded a second time drifts from the hostname the
moment a robot is renamed, and the disagreement is invisible until a query lands
under a prefix nobody serves.

A card stamped with a schema version this build does not know is refused rather
than read: a field that changed meaning between versions would otherwise be handed
to a running agent. An absent card is refused too — unlike a laptop in client
mode, an agent with no card has no namespace to serve under and nothing to do.
"""

from __future__ import annotations

import json
import os
import platform
from dataclasses import dataclass
from pathlib import Path

#: The only card schema this build reads. Bumped in lockstep with the writer in
#: fm-setup, never guessed past.
SCHEMA_VERSION = 1

#: Robot kinds an adapter exists for. A card naming anything else is refused at
#: start rather than serving a verb set no adapter can answer.
ROBOT_KINDS = ("anvil-openarm-v2", "axol")

ENV_OVERRIDE = "FM_MACHINE_FILE"
LINUX_PATH = Path("/etc/fm/machine.json")
MACOS_RELATIVE = Path("fm/machine.json")


class CardError(Exception):
    """This host's card is absent, unreadable, or not a robot's."""


@dataclass(frozen=True)
class RobotCard:
    """What the agent needs to know about the host it runs on."""

    name: str
    kind: str

    @property
    def namespace(self) -> str:
        """The namespace stem derived from the name (``fm-rob-01`` → ``fm_rob_01``)."""
        return self.name.replace("-", "_")


def card_path() -> Path:
    """Where this host's card lives, whether or not it is there.

    ``FM_MACHINE_FILE`` wins, which is how a test and a rehearsal container point
    at a card outside the system paths — the same override fm-setup's writer and
    fm-tools' reader honour, so the three can never disagree about which file is
    the card.
    """
    override = os.environ.get(ENV_OVERRIDE, "").strip()
    if override:
        return Path(override).expanduser()
    if platform.system() == "Darwin":
        config_home = os.environ.get("XDG_CONFIG_HOME", "").strip()
        base = Path(config_home) if config_home else Path.home() / ".config"
        return base / MACOS_RELATIVE
    return LINUX_PATH


def read_card(path: Path | None = None) -> RobotCard:
    """Read this host's card, or raise :class:`CardError` explaining why not."""
    path = path or card_path()
    if not path.is_file():
        raise CardError(f"{path} does not exist; run `fm machine init` on this host")
    try:
        card = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CardError(f"{path} is not readable as JSON: {exc}") from exc
    if not isinstance(card, dict):
        raise CardError(f"{path} is not a machine identity card")

    version = card.get("schema_version")
    if version != SCHEMA_VERSION:
        raise CardError(f"{path} is schema_version {version!r}; this build reads {SCHEMA_VERSION}")

    role = card.get("role")
    if role != "robot":
        raise CardError(f"{path} declares role {role!r}; the agent runs on a robot")

    name = card.get("name") or ""
    if not isinstance(name, str) or not name:
        raise CardError(f"{path} has no name")

    kind = card.get("robot") or ""
    if kind not in ROBOT_KINDS:
        raise CardError(f"{path} declares robot {kind!r}; expected one of {ROBOT_KINDS}")

    return RobotCard(name=name, kind=kind)
