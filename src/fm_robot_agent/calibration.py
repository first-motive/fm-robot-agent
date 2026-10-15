"""This robot's camera calibration, kept beside its description bundle.

One source for everyone who places a camera's pixels in 3D: the desktop's twin,
fm-twin's calibration scores and point cloud, and perception. Each camera has a
lens (OpenCV fisheye intrinsics) and a mount (its optical frame's pose on a
link), stored as JSON under the bundle:

    <bundle>/calibration/<camera>.json        {"lens": {...}, "mount": {...}}
    <bundle>/calibration/<camera>.json.prev   the file it replaced

    fm/robot/<ns>/calibration   {"action": "get"}                         every camera
                                {"action": "set", "camera": ..., "lens": ..., "mount": ...}

A write is a caller's data, so every field is checked for shape and range
before it reaches disk, and the previous file is kept so a bad calibration is
one rename from undone.
"""

from __future__ import annotations

import json
import math
import os
import re
from pathlib import Path

from fm_robot_agent.protocol import AdapterError

CALIBRATION_DIR = "calibration"

#: A camera's name, as the loader names it (`cam_chest`), and a link's.
CAMERA_PATTERN = re.compile(r"^cam_[a-z0-9_]{1,32}$")
LINK_PATTERN = re.compile(r"^[A-Za-z0-9_]{1,64}$")

#: What a lens may be. Only the model the cameras use is accepted.
LENS_MODELS = ("fisheye",)

#: Ceiling on one calibration file, read or written.
MAX_FILE_BYTES = 64 << 10


def _number(value: object, low: float, high: float, name: str) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
        raise AdapterError(f"{name} must be a number")
    if not low <= value <= high:
        raise AdapterError(f"{name} must be between {low} and {high}")
    return float(value)


def _vector(value: object, size: int, bound: float, name: str) -> list[float]:
    if not isinstance(value, list) or len(value) != size:
        raise AdapterError(f"{name} must be {size} numbers")
    return [_number(v, -bound, bound, f"{name}[{i}]") for i, v in enumerate(value)]


def check_lens(lens: object) -> dict:
    """A lens as stored, or raise naming the first field that is wrong."""
    if not isinstance(lens, dict):
        raise AdapterError("lens must be an object")
    model = lens.get("model")
    if model not in LENS_MODELS:
        raise AdapterError(f"lens model must be one of {LENS_MODELS}")
    width = _number(lens.get("width"), 1, 10000, "lens width")
    height = _number(lens.get("height"), 1, 10000, "lens height")
    return {
        "model": model,
        "width": int(width),
        "height": int(height),
        "fx": _number(lens.get("fx"), 1, 100000, "lens fx"),
        "fy": _number(lens.get("fy"), 1, 100000, "lens fy"),
        "cx": _number(lens.get("cx"), 0, width, "lens cx"),
        "cy": _number(lens.get("cy"), 0, height, "lens cy"),
        "distortion": _vector(lens.get("distortion"), 4, 10, "lens distortion"),
        "rms_px": _number(lens.get("rms_px", 0), 0, 100, "lens rms_px"),
    }


def check_mount(mount: object) -> dict:
    """A mount as stored: the optical frame's pose on a link, metres and radians."""
    if not isinstance(mount, dict):
        raise AdapterError("mount must be an object")
    parent = mount.get("parent")
    if not isinstance(parent, str) or not LINK_PATTERN.match(parent):
        raise AdapterError("mount parent must be a link name")
    return {
        "parent": parent,
        "xyz": _vector(mount.get("xyz"), 3, 5, "mount xyz"),
        "rpy": _vector(mount.get("rpy"), 3, 2 * math.pi, "mount rpy"),
    }


class CalibrationStore:
    """The calibration files under a description bundle."""

    def __init__(self, bundle_root: Path) -> None:
        self.dir = bundle_root / CALIBRATION_DIR

    def _path(self, camera: str) -> Path:
        if not CAMERA_PATTERN.match(camera):
            raise AdapterError("camera must be a camera name, e.g. cam_chest")
        return self.dir / f"{camera}.json"

    def all(self) -> dict[str, dict]:
        """Every calibrated camera, by name. Unreadable files are skipped, not served."""
        cameras: dict[str, dict] = {}
        if not self.dir.is_dir():
            return cameras
        for path in sorted(self.dir.glob("cam_*.json")):
            camera = path.stem
            if not CAMERA_PATTERN.match(camera) or path.is_symlink():
                continue
            try:
                raw = path.read_bytes()[: MAX_FILE_BYTES + 1]
                if len(raw) > MAX_FILE_BYTES:
                    continue
                data = json.loads(raw)
                entry = {}
                if "lens" in data:
                    entry["lens"] = check_lens(data["lens"])
                if "mount" in data:
                    entry["mount"] = check_mount(data["mount"])
            except (OSError, ValueError, AdapterError):
                continue
            cameras[camera] = entry
        return cameras

    def set(self, camera: str, lens: object = None, mount: object = None) -> dict:
        """Replace a camera's lens, mount, or both; keep the file it replaces."""
        path = self._path(camera)
        if lens is None and mount is None:
            raise AdapterError("set needs a lens, a mount, or both")
        entry = dict(self.all().get(camera, {}))
        if lens is not None:
            entry["lens"] = check_lens(lens)
        if mount is not None:
            entry["mount"] = check_mount(mount)
        body = json.dumps(entry, indent=2, sort_keys=True).encode()
        try:
            self.dir.mkdir(parents=True, exist_ok=True)
            if path.exists():
                os.replace(path, path.with_suffix(".json.prev"))
            temporary = path.with_suffix(".json.tmp")
            temporary.write_bytes(body)
            os.replace(temporary, path)
        except OSError:
            raise AdapterError(f"calibration for {camera} could not be written") from None
        return entry
