"""Camera calibration: stored per camera beside the description, checked on the way in."""

from __future__ import annotations

import json

import pytest

from fm_robot_agent.description import DescriptionBundle
from fm_robot_agent.fake import FakeAdapter
from fm_robot_agent.verbs import KEY_PREFIX, answer, verb_of

NS = "fm_rob_01"
CHEST_LENS = {
    "model": "fisheye", "width": 1920, "height": 1080,
    "fx": 740.244, "fy": 750.103, "cx": 958.504, "cy": 573.152,
    "distortion": [0.111681, -0.028494, 0.0, 0.0], "rms_px": 0.8,
}
CHEST_MOUNT = {"parent": "follower_body_link0", "xyz": [0.087781, -0.008784, 0.555218],
               "rpy": [-2.466778, -0.007809, -1.579296]}


def ask(tmp_path, payload):
    return answer(f"{KEY_PREFIX}/{NS}/calibration", FakeAdapter(), NS,
                  payload=json.dumps(payload).encode(), bundle=DescriptionBundle(tmp_path))


def body(reply):
    return json.loads(reply.payload)


def test_set_then_get_round_trips(tmp_path):
    assert ask(tmp_path, {"action": "set", "camera": "cam_chest", "lens": CHEST_LENS, "mount": CHEST_MOUNT}).ok
    cameras = body(ask(tmp_path, {"action": "get"}))["cameras"]
    assert cameras["cam_chest"]["lens"]["fx"] == pytest.approx(740.244)
    assert cameras["cam_chest"]["mount"]["parent"] == "follower_body_link0"


def test_setting_one_half_keeps_the_other(tmp_path):
    ask(tmp_path, {"action": "set", "camera": "cam_chest", "lens": CHEST_LENS, "mount": CHEST_MOUNT})
    moved = dict(CHEST_MOUNT, xyz=[0.09, 0.0, 0.55])
    ask(tmp_path, {"action": "set", "camera": "cam_chest", "mount": moved})
    chest = body(ask(tmp_path, {"action": "get"}))["cameras"]["cam_chest"]
    assert chest["lens"]["fx"] == pytest.approx(740.244) and chest["mount"]["xyz"] == [0.09, 0.0, 0.55]


def test_the_replaced_file_is_kept(tmp_path):
    ask(tmp_path, {"action": "set", "camera": "cam_chest", "lens": CHEST_LENS})
    ask(tmp_path, {"action": "set", "camera": "cam_chest", "lens": dict(CHEST_LENS, fx=741.0)})
    previous = json.loads((tmp_path / "calibration" / "cam_chest.json.prev").read_text())
    assert previous["lens"]["fx"] == pytest.approx(740.244)


@pytest.mark.parametrize("payload, error", [
    ({"action": "set", "camera": "../etc", "lens": CHEST_LENS}, "camera"),
    ({"action": "set", "camera": "cam_chest"}, "lens, a mount"),
    ({"action": "set", "camera": "cam_chest", "lens": dict(CHEST_LENS, model="pinhole")}, "model"),
    ({"action": "set", "camera": "cam_chest", "lens": dict(CHEST_LENS, fx=-1)}, "fx"),
    ({"action": "set", "camera": "cam_chest", "lens": dict(CHEST_LENS, cx=5000)}, "cx"),
    ({"action": "set", "camera": "cam_chest", "lens": dict(CHEST_LENS, distortion=[1, 2])}, "distortion"),
    ({"action": "set", "camera": "cam_chest", "lens": dict(CHEST_LENS, fy=float("nan"))}, "fy"),
    ({"action": "set", "camera": "cam_chest", "mount": dict(CHEST_MOUNT, parent="a/b")}, "parent"),
    ({"action": "set", "camera": "cam_chest", "mount": dict(CHEST_MOUNT, xyz=[0, 0, 99])}, "xyz"),
    ({"action": "erase"}, "action"),
])
def test_a_bad_write_is_refused_and_named(tmp_path, payload, error):
    reply = ask(tmp_path, payload)
    assert reply.ok is False and error in body(reply)["error"]
    assert not (tmp_path / "calibration" / "cam_chest.json").exists()


def test_an_unreadable_file_is_not_served(tmp_path):
    (tmp_path / "calibration").mkdir()
    (tmp_path / "calibration" / "cam_waist.json").write_text("{not json")
    assert body(ask(tmp_path, {"action": "get"}))["cameras"] == {}


def test_a_wildcard_cannot_read_or_write_calibration():
    assert verb_of(f"{KEY_PREFIX}/*/calibration", NS) is None
