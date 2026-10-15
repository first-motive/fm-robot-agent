"""This robot's URDF and meshes, served from a bundle on its own host.

The desktop draws a robot from its URDF, posed by its ``joint_states``. A
visualiser on the robot's ROS graph gets both from ``robot_state_publisher`` and
a Foxglove bridge; the fleet reaches a robot through the router alone, so the
agent serves the same two things as read verbs:

    fm/robot/<ns>/description     the URDF, as text
    fm/robot/<ns>/asset?uri=...   one mesh the URDF names, as bytes

Both come from one directory on this host, staged once per robot:

    <bundle>/robot.urdf
    <bundle>/packages/<package>/<path>    what package://<package>/<path> names
    <bundle>/<path>                       what a plain relative <path> names

A bundle rather than a live read, because the three robots share nothing that
could serve one: the Axol and the SO-101 have no ROS graph, and the Anvil's
meshes live inside its loader image. Staging copies are listed in the README.

A URI is a caller's string, so it is resolved inside the bundle and refused if
it would leave it, if the file it reaches is not a mesh, or if that file is
larger than any mesh a robot ships. Refusals name the URI, never a host path:
they go to whoever asked.
"""

from __future__ import annotations

import os
import threading
from pathlib import Path

from fm_robot_agent.protocol import AdapterError

BUNDLE_ENV = "FM_ROBOT_DESCRIPTION_DIR"

URDF_NAME = "robot.urdf"
PACKAGES_DIR = "packages"

#: What an asset may be. Every mesh loader the desktop has reads one of these.
MESH_SUFFIXES = (".stl", ".dae", ".obj")

#: Ceiling on one asset. OpenArm's largest visual mesh is ~11 MB; anything past
#: this is not a mesh a robot ships.
MAX_ASSET_BYTES = 32 << 20

#: Ceiling on a URDF. A bimanual OpenArm description is ~100 KB.
MAX_URDF_BYTES = 4 << 20

#: tradeoff: a mesh is read whole into memory and sent as one Zenoh payload, so
#: each read holds up to MAX_ASSET_BYTES. Two at once bounds that at 64 MB; a
#: third caller is refused and retries. Stream in chunks if meshes outgrow it.
MAX_CONCURRENT_ASSETS = 2
_asset_slots = threading.BoundedSemaphore(MAX_CONCURRENT_ASSETS)

PACKAGE_SCHEME = "package://"


def bundle_dir() -> Path:
    """Where this host's bundle lives, whether or not it is there."""
    override = os.environ.get(BUNDLE_ENV, "").strip()
    if override:
        return Path(override).expanduser()
    return Path.home() / ".local" / "share" / "fm-robot-agent" / "description"


def read_bounded(path: Path, limit: int, name: str) -> bytes:
    """Read at most ``limit`` bytes in one open, so a file that grows after it
    was found cannot be read past the ceiling."""
    try:
        with path.open("rb") as handle:
            data = handle.read(limit + 1)
    except OSError:
        raise AdapterError(f"{name} could not be read") from None
    if len(data) > limit:
        raise AdapterError(f"{name} is larger than {limit} bytes")
    return data


class DescriptionBundle:
    """One robot's staged URDF and meshes."""

    def __init__(self, root: Path | None = None) -> None:
        self.root = root or bundle_dir()

    def urdf(self) -> bytes:
        """The URDF, or raise when none is staged."""
        path = self._inside(Path(URDF_NAME))
        if path is None:
            raise AdapterError(f"no description staged: {URDF_NAME} is missing from the bundle")
        return read_bounded(path, MAX_URDF_BYTES, URDF_NAME)

    def asset(self, uri: str) -> bytes:
        """The mesh a URDF URI names, or raise when it is not one this bundle serves."""
        path = self.resolve(uri)
        if path is None:
            raise AdapterError(f"asset {uri!r} is not in the description bundle")
        if not _asset_slots.acquire(blocking=False):
            raise AdapterError("busy serving other assets; ask again")
        try:
            return read_bounded(path, MAX_ASSET_BYTES, f"asset {uri!r}")
        finally:
            _asset_slots.release()

    def resolve(self, uri: str) -> Path | None:
        """The bundle mesh a URI names, or ``None`` when it names none."""
        if "\x00" in uri:
            return None
        if uri.startswith(PACKAGE_SCHEME):
            relative = Path(PACKAGES_DIR) / uri[len(PACKAGE_SCHEME) :]
        elif "://" in uri or uri.startswith("/"):
            return None
        else:
            relative = Path(uri)
        path = self._inside(relative)
        # The suffix is checked on the file reached, not the name asked for, so
        # a link named like a mesh cannot serve the URDF or anything else.
        if path is None or path.suffix.lower() not in MESH_SUFFIXES:
            return None
        return path

    def _inside(self, relative: Path) -> Path | None:
        """A regular file under the bundle, after symlinks, or ``None``.

        Resolved, then checked against the resolved bundle, so neither ``..``
        nor a link inside the bundle can reach past it.
        """
        root = self.root.resolve()
        candidate = (root / relative).resolve()
        if not candidate.is_relative_to(root) or not candidate.is_file():
            return None
        return candidate
