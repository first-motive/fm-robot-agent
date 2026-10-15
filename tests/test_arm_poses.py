"""Calibration poses: the robot's own file, checked before an arm is moved by it."""

# ruff: noqa: F811 — pytest fixtures imported from test_anvil and named as arguments

from __future__ import annotations

import json
import subprocess

import pytest

from fm_robot_agent import anvil, arm_poses, pose_move
from fm_robot_agent.arm_poses import PoseBook, move_request
from fm_robot_agent.protocol import AdapterError

from .test_anvil import adapter, loader  # noqa: F401 — fixtures

BOARD = {f"follower_l_joint{i}": 0.1 * i for i in range(1, 8)} | {f"follower_r_joint{i}": -0.1 * i for i in range(1, 8)}
HOME = {name: 0.0 for name in BOARD}


def stage(root, poses) -> None:
    (root / "calibration").mkdir(parents=True, exist_ok=True)
    (root / "calibration" / "poses.json").write_text(json.dumps({"target": [0.47, 0, 0.207], "poses": poses}))


def test_reads_every_pose(tmp_path):
    stage(tmp_path, {"board-1": BOARD, "home": HOME})
    assert PoseBook(tmp_path).pose("board-1")["follower_l_joint3"] == pytest.approx(0.3)


@pytest.mark.parametrize("poses, error", [
    ({"board-1": BOARD}, "no home"),
    ({"home": {"follower_l_finger_joint1": 0.0}}, "not an arm joint"),
    ({"home": {"follower_l_joint1": 9.0}}, "out of range"),
    ({"home": {"follower_l_joint1": float("nan")}}, "out of range"),
    ({"home": {"follower_l_joint1": True}}, "out of range"),
    ({"home": HOME, "Board 1": BOARD}, "not a pose name"),
    ({"home": {}}, "has no joints"),
])
def test_refuses_a_file_that_does_not_check_out(tmp_path, poses, error):
    stage(tmp_path, poses)
    with pytest.raises(AdapterError, match=error):
        PoseBook(tmp_path).read()


def test_no_file_is_a_clear_refusal(tmp_path):
    with pytest.raises(AdapterError, match="no calibration poses"):
        PoseBook(tmp_path).read()


def test_an_unknown_pose_names_those_there_are(tmp_path):
    stage(tmp_path, {"board-1": BOARD, "home": HOME})
    with pytest.raises(AdapterError, match="board-1, home"):
        PoseBook(tmp_path).pose("board-9")


def test_a_move_is_timed_by_its_slowest_joint_and_never_rushed():
    assert pose_move.plan({"a": 0.5}, {"a": 0.0}, max_speed=0.25, min_s=6) == 6
    assert pose_move.plan({"a": 3.0, "b": 0.1}, {"a": 0.0, "b": 0.0}, max_speed=0.25, min_s=6) == pytest.approx(18)


def test_the_peak_speed_of_the_timed_move_holds_the_cap():
    # A rest-to-rest cubic over T for a swing d: q(t) = d(3s² - 2s³), s = t/T; peak 1.5 d/T at s = ½.
    swing = 3.0
    duration = pose_move.plan({"a": swing}, {"a": 0.0}, max_speed=0.25, min_s=1)
    times = [i / 1000 for i in range(1001)]
    speeds = [abs(swing * (6 * s - 6 * s * s)) / duration for s in times]
    assert max(speeds) == pytest.approx(0.25, rel=1e-3)


@pytest.mark.parametrize("armed, recording, collecting, code", [
    (False, False, False, "not_armed"),
    (True, True, False, "recording"),
    (True, None, False, "recording"),
    (True, False, True, "collecting"),
    (True, False, None, "collecting"),
])
def test_a_move_is_refused_whenever_a_guard_fails_or_cannot_say(armed, recording, collecting, code):
    assert arm_poses.move_refusal(armed, recording, collecting)[0] == code


def test_a_move_goes_ahead_only_when_every_guard_is_clear():
    assert arm_poses.move_refusal(True, False, False) is None


# --- the Anvil adapter ---------------------------------------------------------


@pytest.fixture
def posed(adapter, loader, monkeypatch):
    """A bundle with poses, a pose arm file, and a stub for the mover's run."""
    monkeypatch.setenv("FM_ROBOT_DESCRIPTION_DIR", str(loader / "bundle"))
    stage(loader / "bundle", {"board-1": BOARD, "home": HOME})
    monkeypatch.setenv(arm_poses.ARM_ENV, str(loader / "pose-move-enabled"))
    (loader / "pose-move-enabled").touch()
    calls = []
    previous = subprocess.run

    def run(argv, **kwargs):
        if "python3 -" in " ".join(argv):
            calls.append(json.loads(argv[-1].rsplit("python3 - ", 1)[1].strip("'")))
            return subprocess.CompletedProcess(argv, 0, json.dumps({"ok": True, "message": "moved in 6.0 s",
                                                                    "duration_s": 6.0}) + "\n", "")
        return previous(argv, **kwargs)

    monkeypatch.setattr(subprocess, "run", run)
    return calls


def test_list_names_the_poses(adapter, posed):
    assert adapter.pose("list").detail["poses"] == ["board-1", "home"]


def test_go_moves_to_the_named_pose_and_holds(adapter, posed):
    outcome = adapter.pose("go", "board-1")
    assert outcome.ok and posed[-1]["targets"] == pytest.approx(BOARD)
    assert posed[-1]["hand_back"] is False and posed[-1]["plan"] is False


def test_home_moves_home_and_hands_the_arms_back(adapter, posed):
    adapter.pose("home")
    assert posed[-1]["targets"] == HOME and posed[-1]["hand_back"] is True


def test_plan_moves_nothing_and_needs_no_arming(adapter, posed, loader):
    (loader / "pose-move-enabled").unlink()
    adapter.pose("plan", "board-1")
    assert posed[-1]["plan"] is True


def test_a_move_is_refused_until_armed_on_the_robot(adapter, posed, loader):
    (loader / "pose-move-enabled").unlink()
    outcome = adapter.pose("go", "board-1")
    assert not outcome.ok and outcome.detail["code"] == "not_armed" and not posed


def test_a_move_is_refused_while_recording(adapter, posed, monkeypatch):
    monkeypatch.setattr(anvil.AnvilAdapter, "_is_recording", lambda self: True)
    outcome = adapter.pose("go", "board-1")
    assert not outcome.ok and outcome.detail["code"] == "recording" and not posed


def test_the_request_carries_the_speed_caps():
    body = json.loads(move_request(HOME, hand_back=True, plan=False))
    assert body["max_speed"] == arm_poses.MAX_SPEED and body["min_s"] == arm_poses.MIN_SECONDS


# --- the mover, against a fake controller layer ---------------------------------


ALL = {name: 0.0 for side in pose_move.ARMS for name in pose_move.joints_of(side)}
GOAL = {"targets": {"follower_l_joint1": 0.5, "follower_r_joint1": -0.5}, "max_speed": 0.25, "min_s": 6,
        "hand_back": False, "plan": False, "deadline_s": 70}


class FakeControllers:
    def __init__(self, **fail):
        self.fail = fail
        self.calls = []
        self.where = dict(ALL)
        self.active = "forward"

    def measure(self, names, timeout_s):
        self.calls.append("measure")
        return None if self.fail.get("measure") else dict(self.where)

    def servers_ready(self, timeout_s):
        return not self.fail.get("servers")

    def switch(self, activate, deactivate):
        self.calls.append(("switch", activate[0].endswith("jtc")))
        if self.fail.get("switch_on") and activate[0].endswith("jtc"):
            return False
        if self.fail.get("switch_back") and not activate[0].endswith("jtc"):
            return False
        self.active = "trajectory" if activate[0].endswith("jtc") else "forward"
        return True

    def send(self, side, names, positions, duration_s):
        self.calls.append(("send", side))
        if self.fail.get("raise") == "send":
            raise RuntimeError("rclpy fell over")
        if self.fail.get("refuse") == side:
            return False
        self.duration = duration_s
        if not self.fail.get("short"):
            self.where.update(zip(names, positions, strict=True))
        return True

    def wait(self, timeout_s):
        self.waited = timeout_s
        if self.fail.get("raise") == "wait":
            raise RuntimeError("rclpy fell over")
        if self.fail.get("timeout"):
            return None
        return [1, 0] if self.fail.get("error") else [0, 0]

    def cancel(self):
        self.calls.append("cancel")

    def seed(self, side, positions):
        self.calls.append(("seed", side))


def test_a_move_reaches_its_target_and_holds():
    c = FakeControllers()
    reply = pose_move.Mover(c).run(GOAL)
    assert reply["ok"] and reply["handed_back"] is False and c.active == "trajectory"
    assert c.where["follower_l_joint1"] == 0.5 and c.duration == pytest.approx(6)


def test_home_hands_the_arms_back_seeded_where_they_are():
    c = FakeControllers()
    reply = pose_move.Mover(c).run({**GOAL, "hand_back": True})
    assert reply["ok"] and reply["handed_back"] and c.active == "forward"
    assert ("seed", "l") in c.calls and ("seed", "r") in c.calls


def test_plan_measures_and_moves_nothing():
    c = FakeControllers()
    reply = pose_move.Mover(c).run({**GOAL, "plan": True})
    assert reply["ok"] and reply["moves"]["follower_l_joint1"] == 0.5
    assert not any(isinstance(call, tuple) and call[0] in ("switch", "send") for call in c.calls)


@pytest.mark.parametrize("fail", [{"measure": True}, {"servers": True}, {"switch_on": True}])
def test_a_failure_before_any_goal_moves_nothing(fail):
    c = FakeControllers(**fail)
    reply = pose_move.Mover(c).run(GOAL)
    assert not reply["ok"] and "nothing moved" in reply["message"]
    assert not any(isinstance(call, tuple) and call[0] == "send" for call in c.calls)


def test_no_goal_is_sent_until_both_servers_answer():
    c = FakeControllers(servers=True)
    pose_move.Mover(c).run(GOAL)
    assert ("send", "l") not in c.calls


@pytest.mark.parametrize("fail", [{"refuse": "r"}, {"timeout": True}, {"error": True}, {"short": True}])
def test_a_failure_after_the_switch_cancels_and_hands_the_arms_back(fail):
    c = FakeControllers(**fail)
    reply = pose_move.Mover(c).run(GOAL)
    assert not reply["ok"] and reply["handed_back"] and c.active == "forward"
    if not fail.get("short"):
        assert "cancel" in c.calls


def test_a_failed_hand_back_says_where_the_arms_are():
    c = FakeControllers(timeout=True, switch_back=True)
    reply = pose_move.Mover(c).run(GOAL)
    assert not reply["ok"] and reply["handed_back"] is False and "trajectory controllers" in reply["message"]


def test_a_move_past_its_deadline_is_not_started():
    c = FakeControllers()
    reply = pose_move.Mover(c).run({**GOAL, "targets": {"follower_l_joint1": 3.4}, "deadline_s": 20})
    assert not reply["ok"] and "nothing moved" in reply["message"] and c.active == "forward"


@pytest.mark.parametrize("where", ["send", "wait"])
def test_an_exception_after_the_switch_still_cancels_and_hands_back(where):
    c = FakeControllers()
    c.fail["raise"] = where
    reply = pose_move.Mover(c).run(GOAL)
    assert not reply["ok"] and reply["handed_back"] and c.active == "forward" and "cancel" in c.calls


def test_every_wait_is_cut_to_the_budget_with_the_reserve_kept():
    # 15 s pass before the wait; the move alone would ask for 6 + 10 = 16 s.
    now = [0.0]
    c = FakeControllers(timeout=True)
    send = c.send

    def slow_send(*args):
        now[0] += 7.5
        return send(*args)

    c.send = slow_send
    reply = pose_move.Mover(c, clock=lambda: now[0]).run({**GOAL, "deadline_s": 40})
    assert c.waited == pytest.approx(40 - pose_move.Mover.RESERVE_S - 15)
    assert reply["handed_back"] and "cancel" in c.calls


def test_a_switch_that_raises_still_hands_the_arms_back():
    c = FakeControllers()
    switch = c.switch
    raised = []

    def flaky(activate, deactivate):
        if activate[0].endswith("jtc") and not raised:
            raised.append(True)
            raise RuntimeError("controller manager fell over")
        return switch(activate, deactivate)

    c.switch = flaky
    reply = pose_move.Mover(c).run(GOAL)
    assert not reply["ok"] and reply["handed_back"] and c.active == "forward"


def test_a_hand_back_with_no_fresh_measure_seeds_where_the_arms_last_were():
    c = FakeControllers(timeout=True)
    measures = iter([dict(ALL), None])
    c.measure = lambda names, timeout_s: next(measures, None)
    reply = pose_move.Mover(c).run(GOAL)
    assert reply["handed_back"] and ("seed", "l") in c.calls


def test_a_switch_asks_only_for_what_is_still_to_change():
    # Holding at a pose: the trajectory controllers are already on.
    holding = {"follower_l_jtc": "active", "follower_r_jtc": "active",
               "follower_l_forward_position_controller": "inactive",
               "follower_r_forward_position_controller": "inactive"}
    jtcs = ["follower_l_jtc", "follower_r_jtc"]
    forwards = ["follower_l_forward_position_controller", "follower_r_forward_position_controller"]
    assert pose_move.switch_needed(holding, jtcs, forwards) == ([], [])
    assert pose_move.switch_needed(holding, forwards, jtcs) == (forwards, jtcs)
    mixed = {**holding, "follower_r_jtc": "inactive"}
    assert pose_move.switch_needed(mixed, jtcs, forwards) == (["follower_r_jtc"], [])


JTCS = ["follower_l_jtc", "follower_r_jtc"]
FORWARDS = ["follower_l_forward_position_controller", "follower_r_forward_position_controller"]
STREAMING = {**{c: "inactive" for c in JTCS}, **{c: "active" for c in FORWARDS}}
HOLDING = {**{c: "active" for c in JTCS}, **{c: "inactive" for c in FORWARDS}}


class Manager:
    """Stands in for the controller manager: states, and switches that may be refused or not take."""

    def __init__(self, states, agree=True, take=True):
        self.states = dict(states) if states is not None else None
        self.agree, self.take, self.requests = agree, take, []

    def read(self):
        return dict(self.states) if self.states is not None else None

    def request(self, on, off):
        self.requests.append((on, off))
        if self.agree and self.take:
            self.states.update({c: "active" for c in on} | {c: "inactive" for c in off})
        return self.agree


def test_a_switch_from_streaming_to_trajectory_is_requested_and_confirmed():
    m = Manager(STREAMING)
    assert pose_move.switch_controllers(m.read, m.request, JTCS, FORWARDS)
    assert m.requests == [(JTCS, FORWARDS)] and m.states == HOLDING


def test_nothing_is_requested_when_the_arms_are_already_there():
    m = Manager(HOLDING)
    assert pose_move.switch_controllers(m.read, m.request, JTCS, FORWARDS) and m.requests == []


def test_a_refused_switch_fails():
    assert not pose_move.switch_controllers(*_manager(STREAMING, agree=False), JTCS, FORWARDS)


def test_a_switch_the_manager_agreed_to_but_did_not_make_fails():
    assert not pose_move.switch_controllers(*_manager(STREAMING, take=False), JTCS, FORWARDS)


def test_unreadable_states_fail_without_a_request():
    m = Manager(None)
    assert not pose_move.switch_controllers(m.read, m.request, JTCS, FORWARDS) and m.requests == []


def _manager(states, **kwargs):
    m = Manager(states, **kwargs)
    return m.read, m.request
