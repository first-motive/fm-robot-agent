"""This robot's camera calibration, kept beside its description bundle.

One source for everyone who places a camera's pixels in 3D: the desktop's twin,
fm-twin's calibration scores and point cloud, and perception. Each camera has a
lens (OpenCV fisheye intrinsics) and a mount (its optical frame's pose on a
link), stored as JSON under the bundle:

    <bundle>/calibration/<camera>.json        {"lens": {...}, "mount": {...}}
    <bundle>/calibration/<camera>.json.prev   the file the last write replaced

    fm/robot/<ns>/calibration   {"action": "get"}                         every camera
                                {"action": "set", "camera": ..., "lens": ..., "mount": ...}

A write moves what every consumer believes about where a camera is, so it needs
local consent like `collect start`: the arm file below must exist on the robot.
Every field is checked for shape and range before it reaches disk, the write is
atomic and durable, and the previous file is kept so a bad calibration is one
rename from undone.
"""

from __future__ import annotations

import json
import math
import os
import re
import tempfile
import threading
from pathlib import Path

from fm_robot_agent.description import read_bounded
from fm_robot_agent.protocol import AdapterError

CALIBRATION_DIR = "calibration"

#: The file whose presence arms `calibration set` on this robot. Absent is off:
#: someone with a shell on the robot decides when calibration may change.
ARM_ENV = "FM_CALIBRATION_ARM_FILE"
DEFAULT_ARM_FILE = Path.home() / ".local" / "state" / "fm-robot-agent" / "calibration-set-enabled"

#: A camera's name, as the loader names it (`cam_chest`), and a link's.
CAMERA_PATTERN = re.compile(r"cam_[a-z0-9_]{1,32}")
LINK_PATTERN = re.compile(r"[A-Za-z0-9_]{1,64}")

#: What a lens may be. Only the model the cameras use is accepted.
LENS_MODELS = ("fisheye",)

#: Ceiling on one calibration file, read or written.
MAX_FILE_BYTES = 64 << 10


def arm_file() -> Path:
    override = os.environ.get(ARM_ENV, "").strip()
    return Path(override).expanduser() if override else DEFAULT_ARM_FILE


def _number(value: object, low: float, high: float, name: str) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise AdapterError(f"{name} must be a number")
    # Compared before converting: an integer too large for a float is out of range,
    # not an overflow.
    if not low <= value <= high or not math.isfinite(float(value)):
        raise AdapterError(f"{name} must be between {low} and {high}")
    return float(value)


def _whole(value: object, low: int, high: int, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or not low <= value <= high:
        raise AdapterError(f"{name} must be a whole number between {low} and {high}")
    return value


def _vector(value: object, size: int, bound: float, name: str) -> list[float]:
    if not isinstance(value, list) or len(value) != size:
        raise AdapterError(f"{name} must be {size} numbers")
    return [_number(v, -bound, bound, f"{name}[{i}]") for i, v in enumerate(value)]


def check_lens(lens: object) -> dict:
    """A lens as stored, or raise naming the first field that is wrong. `rms_px`,
    the solve's reprojection error, is optional: a lens copied from elsewhere may
    not carry one."""
    if not isinstance(lens, dict):
        raise AdapterError("lens must be an object")
    if lens.get("model") not in LENS_MODELS:
        raise AdapterError(f"lens model must be one of {LENS_MODELS}")
    width = _whole(lens.get("width"), 1, 10000, "lens width")
    height = _whole(lens.get("height"), 1, 10000, "lens height")
    return {
        "model": lens["model"],
        "width": width,
        "height": height,
        "fx": _number(lens.get("fx"), 1, 100000, "lens fx"),
        "fy": _number(lens.get("fy"), 1, 100000, "lens fy"),
        "cx": _number(lens.get("cx"), 0, width, "lens cx"),
        "cy": _number(lens.get("cy"), 0, height, "lens cy"),
        "distortion": _vector(lens.get("distortion"), 4, 10, "lens distortion"),
        "rms_px": _number(lens.get("rms_px", 0), 0, 100, "lens rms_px"),
    }


def check_mount(mount: object) -> dict:
    """A mount as stored: the optical frame's pose on a link, metres and radians,
    within ±5 m and ±2π."""
    if not isinstance(mount, dict):
        raise AdapterError("mount must be an object")
    parent = mount.get("parent")
    if not isinstance(parent, str) or not LINK_PATTERN.fullmatch(parent):
        raise AdapterError("mount parent must be a link name")
    return {
        "parent": parent,
        "xyz": _vector(mount.get("xyz"), 3, 5, "mount xyz"),
        "rpy": _vector(mount.get("rpy"), 3, 2 * math.pi, "mount rpy"),
    }


class CalibrationStore:
    """The calibration files under a description bundle."""

    #: Two writers in this process must not interleave a read-modify-write.
    _lock = threading.Lock()

    def __init__(self, bundle_root: Path, arm: Path | None = None) -> None:
        self.dir = bundle_root.resolve() / CALIBRATION_DIR
        self.arm = arm or arm_file()

    def _path(self, camera: str) -> Path:
        if not isinstance(camera, str) or not CAMERA_PATTERN.fullmatch(camera):
            raise AdapterError("camera must be a camera name, e.g. cam_chest")
        return self.dir / f"{camera}.json"

    def _load(self, path: Path) -> dict | None:
        """One camera's stored entry, or None when it is absent, a link, or unreadable."""
        if path.is_symlink() or not path.is_file():
            return None
        try:
            data = json.loads(read_bounded(path, MAX_FILE_BYTES, path.name))
        except (AdapterError, ValueError):
            return None
        if not isinstance(data, dict):
            return None
        try:
            entry = {}
            if "lens" in data:
                entry["lens"] = check_lens(data["lens"])
            if "mount" in data:
                entry["mount"] = check_mount(data["mount"])
        except AdapterError:
            return None
        return entry

    def read_all(self) -> dict[str, dict]:
        """Every calibrated camera, by name. A file that does not check out is skipped."""
        cameras: dict[str, dict] = {}
        if not self.dir.is_dir():
            return cameras
        for path in sorted(self.dir.glob("cam_*.json")):
            if CAMERA_PATTERN.fullmatch(path.stem) and (entry := self._load(path)) is not None:
                cameras[path.stem] = entry
        return cameras

    def write(self, camera: str, lens: object = None, mount: object = None) -> dict:
        """Replace a camera's lens, mount, or both; keep the file it replaces."""
        path = self._path(camera)
        if not self.arm.is_file():
            raise AdapterError("calibration writes are not armed on this robot")
        if lens is None and mount is None:
            raise AdapterError("set needs a lens, a mount, or both")
        checked_lens = check_lens(lens) if lens is not None else None
        checked_mount = check_mount(mount) if mount is not None else None
        with self._lock:
            current = self._load(path)
            if path.exists() and current is None:
                raise AdapterError(f"the stored calibration for {camera} does not check out; restore or remove it")
            entry = dict(current or {})
            if checked_lens is not None:
                entry["lens"] = checked_lens
            if checked_mount is not None:
                entry["mount"] = checked_mount
            self._replace(path, json.dumps(entry, indent=2, sort_keys=True).encode())
        print(f"fm-robot-agent: calibration {camera}: {current} -> {entry}", flush=True)
        return entry

    def _replace(self, path: Path, body: bytes) -> None:
        """Write durably, keep the old file, and swap the new one in only when it is whole."""
        try:
            self.dir.mkdir(parents=True, exist_ok=True)
            fd, temporary = tempfile.mkstemp(dir=self.dir, prefix=".write-")
            try:
                with os.fdopen(fd, "wb") as handle:
                    handle.write(body)
                    handle.flush()
                    os.fsync(handle.fileno())
                if path.exists():
                    previous = path.with_suffix(".json.prev")
                    previous.unlink(missing_ok=True)
                    os.link(path, previous)
                os.replace(temporary, path)
            finally:
                Path(temporary).unlink(missing_ok=True)
            directory = os.open(self.dir, os.O_RDONLY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        except OSError:
            raise AdapterError(f"calibration for {path.stem} could not be written") from None
