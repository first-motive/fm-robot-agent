"""What a configuration key is, and what may be done to it.

A robot's configuration is not one kind of thing. Some keys move a joint, some
sever the telemetry the fleet watches the robot through, and most do neither.
Writing all of them the same way is what turns a convenience verb into a way to
reproduce, from a Mac, the three silent transport defects the 2026-09-01 hardware
run found. So every key carries a class, and the class decides the guard:

    severing   write, restart, watch the data plane, revert if it stays dead
    motion     refused unless the robot reports itself idle
    tuning     written through
    mode       validated against what the robot actually offers
    unknown    readable, never writable

The classes live here rather than in an adapter because both robots have them and
the verb router renders either one with the same code. What each adapter supplies
is its own table: the Anvil's is ours, because ``.env.config`` ships no schema;
the Axol's is Almond's, which the adapter forwards.

:class:`Journal` is the other half of the severing guard. A change that may kill
the agent's own telemetry has to survive the agent: the previous contents go to
disk before anything is written, so an agent that restarts mid-change finds the
open window and finishes the rollback it inherited.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path

#: A key whose value moves the network the fleet watches this robot through.
SEVERING = "severing"
#: A key that shapes how the arms move. Refused unless the robot is idle.
MOTION = "motion"
#: A key that changes behaviour without either risk. Written through.
TUNING = "tuning"
#: The key that selects which control configuration the robot runs.
MODE = "mode"
#: A key the vendor added and nobody has classified. Readable, never writable.
UNKNOWN = "unknown"

CLASSES = (SEVERING, MOTION, TUNING, MODE, UNKNOWN)

#: Everything except :data:`UNKNOWN`. An unclassified key is refused for writing
#: because ``.env.config`` is an ``env_file`` for a container that runs
#: ``privileged: true`` — writing an unknown key there is environment injection,
#: not a configuration edit.
WRITABLE_CLASSES = (SEVERING, MOTION, TUNING, MODE)

#: The key name a caller may use for "whichever key selects this robot's mode".
#: The two robots spell it differently — a YAML filename on the Anvil, an
#: operation on the Axol — and `fm robot <name> mode <value>` predates both, so
#: each adapter resolves this alias to its own key.
MODE_ALIAS = "mode"


class ConfigError(Exception):
    """A configuration write that must not happen. Carries the reason verbatim."""


@dataclass(frozen=True)
class Setting:
    """One key of a robot's configuration, as the fleet sees it.

    Value and class together are the whole reply for one key: the class is what
    lets a caller — a CLI, a desktop form that does not exist yet — show which
    keys it may write and which guard would refuse it, without knowing anything
    about either robot.
    """

    key: str
    value: object
    klass: str
    #: What a caller may type. Empty means anything the validator accepts.
    options: tuple[str, ...] = ()
    #: The vendor's own one-line description, when the vendor ships one.
    help: str = ""

    @property
    def writable(self) -> bool:
        return self.klass in WRITABLE_CLASSES

    def as_dict(self) -> dict:
        """The wire shape. ``class`` is a keyword in Python and not on the wire."""
        return {
            "key": self.key,
            "value": self.value,
            "class": self.klass,
            "writable": self.writable,
            "options": list(self.options),
            "help": self.help,
        }


@dataclass
class Journal:
    """The one pending severing change, on disk beside the robot's config.

    A severing write is the only change that can take away the channel its own
    result would come back on. It is journalled before it is applied and cleared
    only once the data plane has been seen alive, so the two ways it can go wrong
    both end at the same place: telemetry stays dead and the agent reverts, or the
    agent dies and the next one reverts on startup.

    One window at a time. A second severing write while one is open is refused
    rather than queued — two overlapping reverts would restore each other's
    intermediate state.
    """

    path: Path
    #: File contents to restore, keyed by absolute path. Whole files, not lines:
    #: a revert has to undo a paired write across two files, and the smallest
    #: thing that is certainly correct is what was there before.
    files: dict[str, str] = field(default_factory=dict)

    def open(self, key: str, value: str, files: dict[Path, str]) -> None:
        """Record what is about to change, before any of it is written."""
        entry = {
            "key": key,
            "value": value,
            "files": {str(path): text for path, text in files.items()},
        }
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(self.path.suffix + ".tmp")
        tmp.write_text(json.dumps(entry, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        os.replace(tmp, self.path)

    def read(self) -> dict | None:
        """The open change, or ``None`` when none is open."""
        try:
            entry = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return None
        if not isinstance(entry, dict) or not isinstance(entry.get("files"), dict):
            return None
        return entry

    def close(self) -> None:
        """Drop the open change. The write is kept, or has already been undone."""
        try:
            self.path.unlink()
        except OSError:
            pass


def restore(files: dict) -> list[str]:
    """Put journalled file contents back, and report which paths were restored.

    Best effort per file on purpose: a revert that stops at the first failure
    leaves the pair half-restored, which is the state the pairing exists to make
    impossible.
    """
    restored = []
    for path, text in files.items():
        try:
            target = Path(path)
            tmp = target.with_suffix(target.suffix + ".tmp")
            tmp.write_text(text, encoding="utf-8")
            os.replace(tmp, target)
            restored.append(path)
        except OSError:
            continue
    return restored
