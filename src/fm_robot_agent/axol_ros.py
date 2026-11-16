"""The fm-axol ROS stack, as two systemd units the agent starts and stops.

fm-axol installs both units and a polkit rule that lets the robot's login user
start, stop and query exactly these two without sudo, so every call here is a
plain ``systemctl``. Neither unit is enabled at boot: the stack runs only while
the Axol adapter's ``ros`` mode is on.

    fm-axol-core.service   the host core, the one process on the CAN buses
    fm-axol-ros.service    the Jazzy container, through docker compose

The core starts first and stops last, because the container only talks to it.
A start that leaves either unit short of ``active`` is a failed start, and both
are stopped again, so a half-started stack never holds the buses.
"""

from __future__ import annotations

import subprocess

CORE_UNIT = "fm-axol-core.service"
ROS_UNIT = "fm-axol-ros.service"

#: Start order. Stops run it backwards.
UNITS = (CORE_UNIT, ROS_UNIT)

#: What a state read reports when ``systemctl`` hangs or answers nothing.
UNRESPONSIVE = "unresponsive"

#: What a state read reports on a host with no ``systemctl`` at all, which
#: cannot be running either unit.
ABSENT = "absent"

#: The only states that hold no process on the CAN buses. Everything else holds
#: them: ``activating`` and ``deactivating`` are a core still opening or closing
#: the buses, and a state nobody could read may be a running core.
IDLE_STATES = ("inactive", "failed", ABSENT)

#: A start waits for the unit to come up, and the container's compose pull and
#: controller spawn are the slow part. A stop is bounded the same way.
SYSTEMCTL_TIMEOUT_S = 90

#: A state read is answered from systemd's memory and waits on nothing.
STATE_TIMEOUT_S = 10


def holds_buses(states: dict[str, str]) -> bool:
    """Whether any unit in a :meth:`RosStack.states` answer may command the buses."""
    return any(state not in IDLE_STATES for state in states.values())


class RosStack:
    """Start, stop and read the two fm-axol units."""

    def states(self) -> dict[str, str]:
        """Each unit's ``systemctl is-active`` answer, in start order."""
        return {unit: self._state(unit) for unit in UNITS}

    def active(self) -> bool:
        """Whether any unit may be commanding the buses."""
        return holds_buses(self.states())

    def start(self) -> str | None:
        """Start the core, then the container. ``None`` on success, else the reason.

        A failure stops both units before it returns, whichever got started.
        """
        for unit in UNITS:
            failure = self._systemctl("start", unit)
            if failure is None and (state := self._state(unit)) != "active":
                failure = f"{unit} is {state} after start"
            if failure is not None:
                self.stop()
                return failure
        return None

    def stop(self) -> str | None:
        """Stop the container, then the core. ``None`` on success, else the reasons.

        Both stops are attempted even when the first fails: the core is the one
        on the buses, and a stuck container is no reason to leave it running.
        """
        failures = [failure for unit in reversed(UNITS) if (failure := self._systemctl("stop", unit))]
        return "; ".join(failures) or None

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
        """Run one start or stop, returning ``None`` or why it failed."""
        try:
            done = subprocess.run(
                ["systemctl", action, unit],
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
            return f"systemctl {action} {unit} failed: {reason}"
        return None
