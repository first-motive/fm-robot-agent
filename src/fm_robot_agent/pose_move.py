"""Move both Anvil arms to one set of joint angles, slowly, then hold or hand back.

Not imported by the agent: it is read as text and run inside the loader's ros2
container (`python3 - '<json>'`), where rclpy and the controller messages are.
The agent decides what may move and where to; this script only does it:

    measure the joints; wait for both trajectory servers
    switch both arms to their trajectory controllers (the forward-position ones off)
    one waypoint per arm at the target, timed so no joint passes max_speed
    wait for both to finish, or cancel both at the deadline (the arms then hold)
    hand back: switch to the forward-position controllers, seeded with the measured
               joints as fm-skills' motion node does, so the arm cannot jump

Any failure after the switch cancels both goals and hands the arms back, so they
are never left on controllers nothing commands. The reply says where they are.

Input, one JSON argument:
    {"targets": {"follower_l_joint1": 0.1, ...}, "max_speed": 0.25, "min_s": 6,
     "hand_back": false, "plan": false, "deadline_s": 60}

Output, one JSON line: {"ok": bool, "message": str, "duration_s": float, ...}.
"""

from __future__ import annotations

import json
import sys
import time
from typing import Protocol

ARMS = {
    "l": {"jtc": "follower_l_jtc", "forward": "follower_l_forward_position_controller"},
    "r": {"jtc": "follower_r_jtc", "forward": "follower_r_forward_position_controller"},
}
#: A single waypoint ending at rest is interpolated as a cubic, whose peak speed
#: is 1.5 times its average; the move is stretched by this so the peak holds the cap.
CUBIC_PEAK = 1.5
#: A joint further than this from its target after the move has not reached it.
REACH_TOLERANCE = 0.05


def joints_of(side: str) -> list[str]:
    """A trajectory controller's joints, in its own order: seven, then the finger."""
    return [f"follower_{side}_joint{i}" for i in range(1, 8)] + [f"follower_{side}_finger_joint1"]


def switch_needed(states: dict[str, str], activate: list[str], deactivate: list[str]) -> tuple[list[str], list[str]]:
    """Only the changes still to make: a controller already active is not
    activated again, nor an inactive one deactivated. A strict switch refuses
    either, so asking for them fails every move after the first."""
    return ([c for c in activate if states.get(c) != "active"],
            [c for c in deactivate if states.get(c) == "active"])


def switch_controllers(read_states, request, activate: list[str], deactivate: list[str]) -> bool:
    """Bring the controllers to the wanted states, asking only for what is still
    to change, and confirm they got there.

    `read_states()` gives each controller's state, or None; `request(on, off)`
    asks the controller manager for a strict switch and says whether it agreed.
    tradeoff: a controller mid-transition ("activating", "deactivating") reads as
    not active; the move then asks again and a strict switch may refuse, which
    fails the move with nothing moved rather than guessing.
    """
    before = read_states()
    if before is None:
        return False
    on, off = switch_needed(before, activate, deactivate)
    if not on and not off:
        return True
    if not request(on, off):
        return False
    # The manager's ok says it accepted the request, not that every controller
    # reached its state, so the states are read again.
    after = read_states() or {}
    return all(after.get(c) == "active" for c in on) and all(after.get(c) != "active" for c in off)


def plan(targets: dict[str, float], measured: dict[str, float], max_speed: float, min_s: float) -> float:
    """How long the move takes: long enough that the fastest joint's peak speed
    stays at max_speed, and never under min_s."""
    largest = max(abs(targets[name] - measured[name]) for name in targets)
    return max(min_s, CUBIC_PEAK * largest / max_speed)


class Controllers(Protocol):
    """What a move needs of the robot, so the move itself can be tested."""

    def measure(self, names: set[str], timeout_s: float) -> dict[str, float] | None: ...
    def servers_ready(self, timeout_s: float) -> bool: ...
    def switch(self, activate: list[str], deactivate: list[str]) -> bool: ...
    def send(self, side: str, names: list[str], positions: list[float], duration_s: float) -> bool: ...
    def wait(self, timeout_s: float) -> list[int] | None: ...
    def cancel(self) -> None: ...
    def seed(self, side: str, positions: list[float]) -> None: ...


class Mover:
    """One move, inside one time budget.

    Every step is given only the time left before the deadline, less a reserve
    kept for cancelling and handing back; the container's own `timeout` is the
    backstop, never the plan. Anything that goes wrong after the switch, an
    exception included, cancels both goals and hands the arms back.
    """

    #: Kept back from the deadline for cancelling, measuring, switching back and seeding.
    RESERVE_S = 20.0

    def __init__(self, controllers: Controllers, clock=time.monotonic) -> None:
        self.c = controllers
        self.clock = clock

    def _left(self, step_s: float) -> float:
        """A step's timeout: its own, or what the budget has left, whichever is less."""
        return max(0.0, min(step_s, self.end - self.RESERVE_S - self.clock()))

    def run(self, goal: dict) -> dict:
        self.end = self.clock() + float(goal.get("deadline_s", 60.0))
        targets = {str(k): float(v) for k, v in goal["targets"].items()}
        wanted = set(targets) | {name for side in ARMS for name in joints_of(side)}
        measured = self.c.measure(wanted, self._left(5.0))
        if measured is None:
            return {"ok": False, "message": "no joint states from both arms; nothing moved"}
        duration = plan(targets, measured, float(goal.get("max_speed", 0.25)), float(goal.get("min_s", 6.0)))
        if goal.get("plan"):
            moves = {name: round(targets[name] - measured[name], 4) for name in targets}
            return {"ok": True, "message": f"would move for {duration:.1f} s", "duration_s": round(duration, 1),
                    "moves": moves}
        if duration + 10 > self._left(float("inf")):
            return {"ok": False, "message": f"the move would take {duration:.0f} s, past its deadline; nothing moved"}
        if not self.c.servers_ready(self._left(5.0)):
            return {"ok": False, "message": "the trajectory controllers did not answer; nothing moved"}

        jtcs = [arm["jtc"] for arm in ARMS.values()]
        forwards = [arm["forward"] for arm in ARMS.values()]
        try:
            if not self.c.switch(jtcs, forwards):
                return {"ok": False, "message": "the trajectory controllers could not be switched on; nothing moved"}
            return self._move(goal, targets, measured, wanted, duration, jtcs, forwards)
        except BaseException as error:
            self._recover()
            handed = self._hand_back(jtcs, forwards, wanted, measured, f"the move failed ({type(error).__name__})")
            if not isinstance(error, Exception):
                raise
            return {"ok": False, **handed}

    def _move(self, goal, targets, measured, wanted, duration, jtcs, forwards) -> dict:
        sent = all(self.c.send(side, joints_of(side), [targets.get(n, measured[n]) for n in joints_of(side)],
                               duration) for side in ARMS)
        codes = self.c.wait(self._left(duration + 10)) if sent else None
        if not sent or codes is None or any(code != 0 for code in codes):
            self._recover()
            why = ("a controller refused the move" if not sent else
                   "the move did not finish in time" if codes is None else f"a controller reported error {codes}")
            return {"ok": False, **self._hand_back(jtcs, forwards, wanted, measured, why)}
        after = self.c.measure(wanted, 2.0) or measured
        off = {name: round(after[name] - targets[name], 4) for name in targets}
        if max(abs(v) for v in off.values()) > REACH_TOLERANCE:
            return {"ok": False, "error_rad": off,
                    **self._hand_back(jtcs, forwards, wanted, after, "a joint stopped short of its target")}
        reply = {"ok": True, "duration_s": round(duration, 1), "error_rad": off}
        if goal.get("hand_back"):
            handed = self._hand_back(jtcs, forwards, wanted, after, "")
            return {**reply, **handed, "ok": handed["handed_back"],
                    "message": f"moved in {duration:.1f} s" + ("" if handed["handed_back"] else
                                                               "; " + handed["message"])}
        return {**reply, "handed_back": False, "message": f"moved in {duration:.1f} s; holding"}

    def _recover(self) -> None:
        """Cancel both goals, whatever else fails: the arms then hold where they are."""
        try:
            self.c.cancel()
        except Exception as error:  # noqa: BLE001 — a failed cancel must not stop the hand-back
            print(f"pose move: cancel failed: {error}", file=sys.stderr, flush=True)

    def _hand_back(self, jtcs, forwards, wanted, last_known: dict[str, float], why: str) -> dict:
        """Give the arms back to the forward controllers, seeded where they are
        (or, if they cannot be measured now, where they last were)."""
        try:
            now = self.c.measure(wanted, 2.0) or last_known
            switched = self.c.switch(forwards, jtcs)
        except Exception:  # noqa: BLE001 — report the arms' state rather than raise past it
            switched = False
        if not switched:
            state = "arms holding on their trajectory controllers; send pose home or restart the stack"
            return {"handed_back": False, "message": f"{why}; {state}" if why else state}
        for side in ARMS:
            self.c.seed(side, [now[name] for name in joints_of(side)])
        return {"handed_back": True, "message": f"{why}; arms handed back where they stopped" if why else "handed back"}


class RosControllers:
    """The robot's controllers through rclpy."""

    #: Seed commands sent after a hand-back, 20 ms apart, as fm-skills' motion node does.
    SEED_REPEATS = 25

    def __init__(self) -> None:
        import rclpy
        from control_msgs.action import FollowJointTrajectory
        from controller_manager_msgs.srv import ListControllers, SwitchController
        from rclpy.action import ActionClient
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray

        self.rclpy = rclpy
        self.FollowJointTrajectory = FollowJointTrajectory
        self.SwitchController = SwitchController
        self.ListControllers = ListControllers
        self.Float64MultiArray = Float64MultiArray
        rclpy.init()
        self.node = rclpy.create_node("fm_robot_agent_pose")
        self.joints: dict[str, float] = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.switcher = self.node.create_client(SwitchController, "/controller_manager/switch_controller")
        self.lister = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        self.clients = {side: ActionClient(self.node, FollowJointTrajectory, f"/{arm['jtc']}/follow_joint_trajectory")
                        for side, arm in ARMS.items()}
        self.publishers = {side: self.node.create_publisher(Float64MultiArray, f"/{arm['forward']}/commands", 10)
                           for side, arm in ARMS.items()}
        self.handles = []

    def _on_joints(self, msg) -> None:
        self.joints.update(zip(msg.name, msg.position, strict=False))

    def _spin_until(self, done, timeout_s: float) -> bool:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if done():
                return True
        return False

    def measure(self, names, timeout_s):
        self.joints.clear()
        return dict(self.joints) if self._spin_until(lambda: names <= set(self.joints), timeout_s) else None

    def servers_ready(self, timeout_s):
        return self.switcher.wait_for_service(timeout_sec=timeout_s) and all(
            client.wait_for_server(timeout_sec=timeout_s) for client in self.clients.values())

    def states(self) -> dict[str, str] | None:
        future = self.lister.call_async(self.ListControllers.Request())
        if not self._spin_until(future.done, 5.0) or future.result() is None:
            return None
        return {c.name: c.state for c in future.result().controller}

    def switch(self, activate, deactivate):
        return switch_controllers(self.states, self._request_switch, activate, deactivate)

    def _request_switch(self, activate, deactivate) -> bool:
        request = self.SwitchController.Request(activate_controllers=activate, deactivate_controllers=deactivate,
                                                strictness=self.SwitchController.Request.STRICT, activate_asap=True)
        future = self.switcher.call_async(request)
        return self._spin_until(future.done, 10.0) and bool(future.result() and future.result().ok)

    def send(self, side, names, positions, duration_s):
        from builtin_interfaces.msg import Duration
        from trajectory_msgs.msg import JointTrajectoryPoint

        goal = self.FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=positions, velocities=[0.0] * len(names),
            time_from_start=Duration(sec=int(duration_s), nanosec=int((duration_s % 1) * 1e9)))]
        future = self.clients[side].send_goal_async(goal)
        if not self._spin_until(future.done, 5.0) or not future.result() or not future.result().accepted:
            return False
        self.handles.append(future.result())
        return True

    def wait(self, timeout_s):
        results = [handle.get_result_async() for handle in self.handles]
        if not self._spin_until(lambda: all(r.done() for r in results), timeout_s):
            return None
        return [r.result().result.error_code for r in results]

    def cancel(self):
        futures = [handle.cancel_goal_async() for handle in self.handles]
        self._spin_until(lambda: all(f.done() for f in futures), 5.0)

    def seed(self, side, positions):
        for _ in range(self.SEED_REPEATS):
            self.rclpy.spin_once(self.node, timeout_sec=0.0)
            self.publishers[side].publish(self.Float64MultiArray(data=positions))
            time.sleep(0.02)

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def main(arg: str) -> dict:
    goal = json.loads(arg)
    controllers = RosControllers()
    try:
        return Mover(controllers).run(goal)
    finally:
        controllers.close()


if __name__ == "__main__":
    try:
        print(json.dumps(main(sys.argv[1])), flush=True)
    except Exception as error:  # noqa: BLE001 — the agent reads one JSON line, whatever went wrong
        print(f"pose move: {type(error).__name__}: {error}", file=sys.stderr, flush=True)
        print(json.dumps({"ok": False, "message": "the pose move failed on the robot; see the agent's journal"}),
              flush=True)
