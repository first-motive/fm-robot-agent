"""What a robot must be able to do, and nothing about how it is reached.

One agent serves two robots that share no code: an Anvil workcell driven through
docker compose and a ROS graph, and an Axol driven through its own FastAPI
process. :class:`RobotAdapter` is the seam between them. Everything above it —
the verb router, the Zenoh service, the tests — is written once against this
protocol; everything below it is vendor-specific and lives in one adapter module.

The verb set is fixed and small on purpose. It is the whole inbound surface the
fleet has to a robot, so every addition widens what a compromised router can ask
for. Motion is not in it: neither robot accepts a trajectory or a command topic
from the fabric, and ``stop`` maps to the vendor's own pause or disconnect rather
than to anything this repo invents.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

from fm_robot_agent.config import Setting

#: Bumped when a payload shape changes meaning. Every reply carries it so a
#: desktop built against an older agent refuses the reply rather than decoding
#: a field that has moved underneath it.
SCHEMA_VERSION = 1


class AdapterError(Exception):
    """The robot refused, or could not be reached. Surfaced to the caller verbatim."""


@dataclass(frozen=True)
class Outcome:
    """The result of a verb that changes something.

    ``ok`` false is a refusal the caller must see, not an exception: a mode that
    does not exist and a stack that is already down are both ordinary answers.
    """

    ok: bool
    message: str = ""
    #: Verb-specific extras merged into the reply — ``episode`` for record,
    #: ``state`` for stop. Kept open rather than modelled per verb, because the
    #: two robots report different things and a third would report a third.
    detail: dict = field(default_factory=dict)


@runtime_checkable
class RobotAdapter(Protocol):
    """One robot's implementation of the fixed verb set."""

    #: The robot kind this adapter drives, matching the card's ``robot`` field.
    kind: str

    def status(self) -> dict:
        """Everything the desktop shows without asking a second question.

        Returns the keys the wire contract names: ``mode``, ``hardware``,
        ``recording``, ``services``, ``disk``. ``kind`` and ``schema_version``
        are added by the router, not by the adapter.
        """

    def up(self) -> Outcome:
        """Bring the robot's stack up."""

    def down(self) -> Outcome:
        """Take the robot's stack down."""

    def set_mode(self, config: str) -> Outcome:
        """Switch control mode. ``config`` is one of the values ``status`` reports.

        Sugar over ``config_write(MODE_ALIAS, config)``, kept because `fm robot
        <name> mode <value>` predates the config verb and still reads better than
        naming the key each robot happens to spell it with.
        """

    def config_read(self) -> list[Setting]:
        """Every key of this robot's own configuration, with its value and class.

        Read from the robot rather than from a table here: both vendors edit
        their configuration files on their own release schedule, and a hardcoded
        key list goes stale silently. What this repo owns is the classification,
        not the enumeration.
        """

    def config_write(self, key: str, value: str) -> Outcome:
        """Write one key, subject to the guard its class carries.

        ``ok`` false is a refusal the caller must see: an unknown key, a motion
        key while the robot is busy, a value the validator rejects.
        """

    def config_rollback(self) -> Outcome:
        """Undo the open severing change, if one is open.

        Called both by the verb and by the agent on startup, so a change whose
        verification the agent did not survive is still undone.
        """

    def record(self, dataset: str, action: str) -> Outcome:
        """Start or stop recording an episode into ``dataset``.

        ``action`` is ``start`` or ``stop``; the router validates that before
        calling. A started episode is returned in ``detail["episode"]``.
        """

    def stop(self) -> Outcome:
        """Halt motion through the vendor's own mechanism.

        The Anvil pauses its hardware state controller; the Axol disconnects its
        motors. Neither is an emergency stop — no e-stop exists on either robot,
        and this does not pretend to be one.
        """

    def episodes(self, dataset: str) -> list[dict]:
        """Episodes recorded into ``dataset``, newest ordering left to the robot."""
