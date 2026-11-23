"""The Axol's lift height: the one piece of robot state this agent keeps on disk.

The Ox cart's lift is set by hand with an allen key and has no encoder, so the
operator's reading off the sticker at the case rim is its only state source. It
is kept in millimetres, 0 fully lowered and the travel fully raised (the vendored
URDF's `lift` limit), in a file fm-axol's install.sh hands to the login user this
agent runs as. fm-axol's container mounts the directory read-only, and its launch
reads the file once, when the ROS stack starts, to place every transform below
the lift. No file means the lift is fully raised.
"""

from __future__ import annotations

import math
import os
import stat
import tempfile
from pathlib import Path

LIFT_TRAVEL_MM = 457.2

#: The path fm-axol's install.sh creates and its compose file mounts. Both repos
#: name it; a move on either side needs the other.
LIFT_FILE = Path("/var/lib/fm-axol/lift/height_mm")

#: A reading is one short number. Anything longer is not one, so it is never read whole.
MAX_FILE_BYTES = 64

#: How much of a refused value a refusal quotes back to the caller.
MAX_ECHO_CHARS = 32


def parse_height(value: object) -> float:
    """A reading in millimetres, inside the travel. Raises ValueError with the reason."""
    shown = repr(value)[:MAX_ECHO_CHARS]
    try:
        height = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"must be a number of mm, not {shown}") from None
    if not (math.isfinite(height) and 0.0 <= height <= LIFT_TRAVEL_MM):
        raise ValueError(f"must be between 0 and {LIFT_TRAVEL_MM} mm, not {shown}")
    return height


def read_height(path: Path) -> float | None:
    """The stored reading. No file, or an empty one, is the full travel: nothing was ever
    entered, and fm-axol's launch reads it the same way. None when the path holds something
    that is not a reading (text, a link, a directory), which fm-axol's launch refuses."""
    if path.is_symlink():
        return None
    try:
        info = path.stat()
    except FileNotFoundError:
        return LIFT_TRAVEL_MM
    except OSError:
        return None
    if not stat.S_ISREG(info.st_mode) or info.st_size > MAX_FILE_BYTES:
        return None
    try:
        text = path.read_text(encoding="utf-8").strip()
    except (OSError, UnicodeDecodeError):
        return None
    if not text:
        return LIFT_TRAVEL_MM
    try:
        return parse_height(text)
    except ValueError:
        return None


def write_height(path: Path, height: float) -> None:
    """Store a parsed reading durably, whole or not at all. Raises OSError when the
    directory is missing or not writable.

    The shape of calibration.py's ``_replace``: a fresh temporary file (mkstemp:
    exclusive, never through a link) beside the target, synced, renamed over it, and the
    directory synced, so a power cut leaves the old reading or the new one, never an
    empty file that would read as fully raised.
    """
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(f"{height}\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(tmp, 0o644)
        os.replace(tmp, path)
    finally:
        Path(tmp).unlink(missing_ok=True)
    directory = os.open(path.parent, os.O_RDONLY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)
