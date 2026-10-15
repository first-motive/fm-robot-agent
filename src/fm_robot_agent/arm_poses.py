"""Named arm poses for camera calibration, kept beside the calibration files.

fm-twin computes poses that aim both wrist cameras at one board on the table
and checks them (`fm-twin poses`); they are staged on the robot as

    <bundle>/calibration/poses.json   {"target": [...], "poses": {"board-1": {...}, "home": {...}}}

and the `pose` verb drives the arms to one by name:

    fm/robot/<ns>/pose   {"action": "list"}               the names
                         {"action": "plan", "name": ...}  how far and how long, nothing moves
                         {"action": "go",   "name": ...}  move there and hold
                         {"action": "home"}               move to "home" and hand the arms back

The fabric names a pose; the angles are this robot's own file, so nothing from
the network becomes a joint command. A move needs local consent like `collect
start` (the arm file below), is refused while recording, and runs slowly.
"""

from __future__ import annotations

import json
import math
import os
import re
from pathlib import Path

from fm_robot_agent.calibration import CALIBRATION_DIR, MAX_FILE_BYTES
from fm_robot_agent.description import read_bounded
from fm_robot_agent.protocol import AdapterError

POSES_FILE = "poses.json"
HOME = "home"

#: The file whose presence arms `pose go` and `pose home` on this robot.
ARM_ENV = "FM_POSE_ARM_FILE"
DEFAULT_ARM_FILE = Path.home() / ".local" / "state" / "fm-robot-agent" / "pose-move-enabled"

POSE_NAME = re.compile(r"[a-z0-9][a-z0-9-]{0,31}")
#: The joints a pose may set: the seven of each Anvil arm. Fingers keep where they are.
JOINT_NAME = re.compile(r"follower_[lr]_joint[1-7]")
#: No Anvil joint turns further than this either way.
MAX_ANGLE = 3.5

#: The slowest joint speed a move is timed by, rad/s, and the least time it takes.
MAX_SPEED = 0.25
MIN_SECONDS = 6.0


def arm_file() -> Path:
    override = os.environ.get(ARM_ENV, "").strip()
    return Path(override).expanduser() if override else DEFAULT_ARM_FILE


class PoseBook:
    """The poses staged on this robot."""

    def __init__(self, bundle_root: Path) -> None:
        self.path = bundle_root.resolve() / CALIBRATION_DIR / POSES_FILE

    def read(self) -> dict[str, dict[str, float]]:
        """Every pose by name, each checked; raise when the file is missing or wrong."""
        if self.path.is_symlink() or not self.path.is_file():
            raise AdapterError("no calibration poses on this robot: stage calibration/poses.json from fm-twin")
        try:
            body = json.loads(read_bounded(self.path, MAX_FILE_BYTES, POSES_FILE))
        except ValueError:
            raise AdapterError(f"{POSES_FILE} is not JSON") from None
        poses = body.get("poses") if isinstance(body, dict) else None
        if not isinstance(poses, dict) or HOME not in poses:
            raise AdapterError(f"{POSES_FILE} has no poses, or no {HOME}")
        return {name: self._check(name, joints) for name, joints in poses.items()}

    @staticmethod
    def _check(name: object, joints: object) -> dict[str, float]:
        if not isinstance(name, str) or not POSE_NAME.fullmatch(name):
            raise AdapterError(f"{POSES_FILE}: {name!r} is not a pose name")
        if not isinstance(joints, dict) or not joints:
            raise AdapterError(f"{POSES_FILE}: pose {name} has no joints")
        out = {}
        for joint, value in joints.items():
            if not isinstance(joint, str) or not JOINT_NAME.fullmatch(joint):
                raise AdapterError(f"{POSES_FILE}: pose {name} sets {joint!r}, which is not an arm joint")
            if isinstance(value, bool) or not isinstance(value, (int, float)) \
                    or not math.isfinite(value) or abs(value) > MAX_ANGLE:
                raise AdapterError(f"{POSES_FILE}: pose {name} sets {joint} out of range")
            out[joint] = float(value)
        return out

    def pose(self, name: str) -> dict[str, float]:
        poses = self.read()
        if name not in poses:
            raise AdapterError(f"no pose {name!r}; this robot has {', '.join(sorted(poses))}")
        return poses[name]


#: How long the mover may take inside the container before it cancels the move
#: (the arms then hold), and the hard stop on its process after that. Both sit
#: under the agent's own timeout, so the process is gone before the lock is let go.
MOVE_DEADLINE_S = 70
PROCESS_DEADLINE_S = 80


def move_request(targets: dict[str, float], hand_back: bool, plan: bool) -> str:
    """The one argument `pose_move.py` reads."""
    return json.dumps({"targets": targets, "max_speed": MAX_SPEED, "min_s": MIN_SECONDS,
                       "hand_back": hand_back, "plan": plan, "deadline_s": MOVE_DEADLINE_S})


def move_refusal(armed: bool, recording: bool | None, collecting: bool | None) -> tuple[str, str] | None:
    """Why a move may not start, as (code, message), or None when it may.
    An unknown recording or loop state refuses: a move is never the guess."""
    if not armed:
        return "not_armed", ("pose moves are not armed on this robot: they move both arms, so an operator "
                             "enables them locally first by creating the pose-move-enabled flag in the agent's "
                             "state directory (see the fm-robot-agent README)")
    if recording is not False:
        return "recording", ("recording: stop the take before moving the arms" if recording
                             else "the recording state could not be read, so the arms are not moved")
    if collecting is not False:
        return "collecting", ("the collection loop is running: stop it before moving the arms" if collecting
                              else "the collection loop's state could not be read, so the arms are not moved")
    return None


def move_script() -> str:
    """`pose_move.py`, as text, to run inside the robot's ROS container."""
    return (Path(__file__).parent / "pose_move.py").read_text()
