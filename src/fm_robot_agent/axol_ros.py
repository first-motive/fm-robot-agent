"""The fm-axol ROS stack, as three systemd units the agent starts and stops.

fm-axol installs the units and ``/etc/sudoers.d/fm-axol``, which lets the
robot's login user run ``/usr/bin/systemctl start|stop|restart`` on exactly
these units, one unit per command, with no password. Starts and stops therefore
run as :data:`PRIVILEGED_SYSTEMCTL` with one unit, so the argv matches a
sudoers line exactly; ``-n`` makes sudo refuse rather than wait for a password
no one will type. ``is-active`` needs no privilege and stays plain
``systemctl``. No unit is enabled at boot: the stack runs only while the Axol
adapter's ``ros`` mode is on.

    fm-axol-core.service   the host core, the one process on the CAN buses
    fm-axol-ros.service    the Jazzy container, through docker compose
    fm-axol-zed.service    the Stereolabs ZED driver container, on the cameras

The core starts first and stops last, because the container only talks to it.
A start that leaves either bus unit short of ``active`` is a failed start, and
both are stopped again, so a half-started stack never holds the buses.

The camera unit is apart from the other two. It starts after the stack is up
and stops before it comes down, and a camera that fails to start fails nothing:
motion needs no camera. A ZED opens in one process, though, so while the unit
runs Almond cannot stream the cameras, and every stop releases them first.
"""

from __future__ import annotations

import subprocess

CORE_UNIT = "fm-axol-core.service"
ROS_UNIT = "fm-axol-ros.service"
CAMERA_UNIT = "fm-axol-zed.service"

#: The units on the CAN buses, in start order. Stops run it backwards.
BUS_UNITS = (CORE_UNIT, ROS_UNIT)

#: Every unit, in start order, as :meth:`RosStack.states` reports them.
UNITS = (*BUS_UNITS, CAMERA_UNIT)

#: The prefix every start and stop runs under. The systemctl path is part of the
#: sudoers match, so it is spelled exactly as fm-axol's rule spells it; sudo's own
#: path is absolute so the service's PATH never picks which sudo runs.
PRIVILEGED_SYSTEMCTL = ("/usr/bin/sudo", "-n", "/usr/bin/systemctl")

#: Where the rule that grants these units lives, named in a refusal.
SUDOERS_RULE = "/etc/sudoers.d/fm-axol"

#: How sudo words a refusal. With ``-n`` and no matching NOPASSWD line it says
#: ``sudo: a password is required``; a user with no rule at all, or with an argv
#: the rule does not match, gets ``Sorry, user ... is not allowed to execute``
#: or ``Sorry, user ... may not run sudo``, which carry no ``sudo:`` prefix.
SUDO_REFUSALS = ("sudo:", "Sorry, user ")

#: What a state read reports when ``systemctl`` hangs or answers nothing.
UNRESPONSIVE = "unresponsive"

#: What a state read reports on a host with no ``systemctl`` at all, which
#: cannot be running either unit.
ABSENT = "absent"

#: The only states that hold no process on the CAN buses or the cameras.
#: Everything else holds them: ``activating`` and ``deactivating`` are a core
#: still opening or closing the buses, and a state nobody could read may be a
#: running core.
IDLE_STATES = ("inactive", "failed", ABSENT)

#: A start waits for the unit to come up, and the container's compose pull and
#: controller spawn are the slow part. A stop is bounded the same way.
SYSTEMCTL_TIMEOUT_S = 90

#: A state read is answered from systemd's memory and waits on nothing.
STATE_TIMEOUT_S = 10


def holds_buses(states: dict[str, str]) -> bool:
    """Whether any bus unit in a :meth:`RosStack.states` answer may command the buses."""
    return any(state not in IDLE_STATES for unit, state in states.items() if unit in BUS_UNITS)


class RosStack:
    """Start, stop and read the fm-axol units."""

    def states(self) -> dict[str, str]:
        """Each unit's ``systemctl is-active`` answer, in start order."""
        return {unit: self._state(unit) for unit in UNITS}

    def active(self) -> bool:
        """Whether any unit may be commanding the buses."""
        return holds_buses(self.states())

    def idle(self) -> bool:
        """Whether no unit holds the buses or the cameras."""
        return all(state in IDLE_STATES for state in self.states().values())

    def start(self) -> str | None:
        """Start the core, then the container. ``None`` on success, else the reason.

        A failure stops both units before it returns, whichever got started, and
        the reason says whether that rollback stopped them: a refused stop leaves
        the core on the buses, and a reply claiming otherwise would hide it.
        """
        for unit in BUS_UNITS:
            failure = self._start_one(unit)
            if failure is not None:
                rollback = self.stop()
                if rollback is not None:
                    return f"{failure}; rollback did not stop the units: {rollback}"
                return f"{failure}; both units stopped"
        return None

    def start_cameras(self) -> str | None:
        """Start the camera unit. ``None`` on success, else the reason.

        Nothing is rolled back on a failure: the caller reports it and carries
        on, because the stack runs without cameras.
        """
        return self._start_one(CAMERA_UNIT)

    def stop(self) -> str | None:
        """Stop the cameras, the container, then the core. ``None`` on success, else the reasons.

        Every stop is attempted even when one fails: the core is the one on the
        buses, and a stuck container is no reason to leave it running. The
        camera unit is stopped only while it runs, so an fm-axol install from
        before the unit existed still stops cleanly.
        """
        units = BUS_UNITS[::-1]
        if self._state(CAMERA_UNIT) not in IDLE_STATES:
            units = (CAMERA_UNIT, *units)
        failures = [failure for unit in units if (failure := self._systemctl("stop", unit))]
        return "; ".join(failures) or None

    def _start_one(self, unit: str) -> str | None:
        """Start one unit and read it back, returning ``None`` or why it is not up."""
        failure = self._systemctl("start", unit)
        if failure is None and (state := self._state(unit)) != "active":
            failure = f"{unit} is {state} after start"
        return failure

    def _state(self, unit: str) -> str:
        try:
            done = subprocess.run(
                ["systemctl", "is-active", unit],
                capture_output=True,
                text=True,
                timeout=STATE_TIMEOUT_S,
                check=False,
            )
        except subprocess.TimeoutExpired:
            return UNRESPONSIVE
        except OSError:
            return ABSENT
        return done.stdout.strip() or UNRESPONSIVE

    def _systemctl(self, action: str, unit: str) -> str | None:
        """Run one start or stop under sudo, returning ``None`` or why it failed."""
        try:
            done = subprocess.run(
                [*PRIVILEGED_SYSTEMCTL, action, unit],
                capture_output=True,
                text=True,
                timeout=SYSTEMCTL_TIMEOUT_S,
                check=False,
            )
        except subprocess.TimeoutExpired:
            return f"systemctl {action} {unit} took longer than {SYSTEMCTL_TIMEOUT_S} s"
        except OSError as exc:
            return f"systemctl {action} {unit} could not run: {exc}"
        if done.returncode != 0:
            reason = (done.stderr or done.stdout).strip() or f"exit {done.returncode}"
            # A refusal by sudo is told apart from a unit that started and failed,
            # so a missing or narrower rule reads as one.
            if reason.startswith(SUDO_REFUSALS):
                return f"sudo refused systemctl {action} {unit}; check {SUDOERS_RULE} from fm-axol's install ({reason})"
            return f"systemctl {action} {unit} failed: {reason}"
        return None
