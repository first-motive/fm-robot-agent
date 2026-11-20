"""This host's First Motive systemd units, read and driven for the ``unit`` verb.

A unit is named by the caller, so the name is the whole attack surface: it is
checked against :data:`FM_UNIT` or the two camera daemons, normalised to
``<name>.service``, and only then placed in an argument list. No shell runs.

    fm-<name>[.service]       list · status · start · stop · restart
    zed_x_daemon.service      list · status · restart
    nvargus-daemon.service    list · status · restart

Reads need no privilege. Changes run as :data:`PRIVILEGED_SYSTEMCTL`, and the
agent's self-restart through ``systemd-run``, both under the robot user's
passwordless sudo; ``-n`` makes sudo refuse rather than wait for a password no
one will type.

The agent's own unit is a special case in both directions. A stop is refused:
nothing could start the agent again from the fabric. A restart is handed to a
transient systemd timer one second out, so this process lives long enough to
send the reply that says the restart is coming.

A start is only a start once the unit has stayed up. ``systemctl start`` returns
when the unit reaches ``active``, and a unit that dies seconds later has still
returned 0, so :func:`settled_state` keeps reading ``is-active`` for
:data:`SETTLE_S` before it calls a start good.

Every subprocess goes through :func:`_run`, which raises :class:`NoAnswer` or
:class:`AdapterError` when the command did not run to an exit and otherwise
hands back the exit for the caller to judge. What systemctl or sudo printed
goes to this host's journal; a reply on the fabric carries a short reason.
"""

from __future__ import annotations

import re
import subprocess
import sys
import time

from fm_robot_agent.protocol import AdapterError

UNIT_ACTIONS = ("list", "status", "start", "stop", "restart")

#: The units this verb may name: every First Motive unit, and nothing else.
FM_UNIT = re.compile(r"^fm-[a-z0-9-]+(\.service)?$")

#: The vendor camera daemons a stuck camera is recovered by restarting. They are
#: the platform's, not ours, so a caller may read and restart them and no more.
CAMERA_DAEMONS = ("zed_x_daemon.service", "nvargus-daemon.service")
CAMERA_DAEMON_ACTIONS = ("status", "restart")

AGENT_UNIT = "fm-robot-agent.service"

#: The prefix every change runs under. Both paths are absolute so the service's
#: PATH never picks which sudo or systemctl runs.
PRIVILEGED_SYSTEMCTL = ("/usr/bin/sudo", "-n", "/usr/bin/systemctl")

#: How the agent restarts itself: a transient timer, outside this unit's cgroup,
#: fires after the reply has gone. A restart run in-process would kill the
#: process before it answered. The fixed name lets a second request see the
#: first is pending, and ``--collect`` unloads it even if the restart failed, so
#: the name is free again afterwards.
RESTART_UNIT = "fm-robot-agent-restart"
DEFERRED_RESTART = ("/usr/bin/sudo", "-n", "/usr/bin/systemd-run", f"--unit={RESTART_UNIT}", "--collect",
                    "--on-active=1", "/usr/bin/systemctl", "restart", AGENT_UNIT)

#: How sudo words a refusal. With ``-n`` and no matching NOPASSWD line it says
#: ``sudo: a password is required``; a user with no rule at all, or with an argv
#: the rule does not match, gets ``Sorry, user ... is not allowed to execute``
#: or ``Sorry, user ... may not run sudo``, which carry no ``sudo:`` prefix.
SUDO_REFUSALS = ("sudo:", "Sorry, user ")

#: What a state read reports when ``systemctl`` hangs or answers nothing.
UNRESPONSIVE = "unresponsive"

#: What a state read reports on a host with no ``systemctl`` at all.
ABSENT = "absent"

#: A start waits for the unit to come up, and a container's compose pull is the
#: slow part. A stop is bounded the same way.
SYSTEMCTL_TIMEOUT_S = 90

#: A state read is answered from systemd's memory and waits on nothing.
STATE_TIMEOUT_S = 10

#: How long a started unit must stay active to count as started, and how often
#: it is read in that time. Long enough to catch a process that exits on a bad
#: config; short enough that a start still answers inside a query's timeout.
SETTLE_S = 3.0
SETTLE_POLL_S = 0.5

#: The properties a unit is described by, and the key each takes on the wire.
SHOWN = {"ActiveState": "active", "SubState": "sub", "UnitFileState": "enabled"}


class NoAnswer(AdapterError):
    """A command ran out of time, as opposed to failing to run at all."""


def _run(argv: list[str] | tuple[str, ...], timeout_s: float) -> subprocess.CompletedProcess:
    """Run one command to its exit. Raises :class:`NoAnswer` on a timeout, :class:`AdapterError` if it cannot run."""
    try:
        return subprocess.run(list(argv), capture_output=True, text=True, timeout=timeout_s, check=False)
    except subprocess.TimeoutExpired as exc:
        raise NoAnswer(f"{' '.join(argv[:2])} did not answer in {timeout_s} s") from exc
    except OSError as exc:
        raise AdapterError(f"{' '.join(argv[:2])} could not run: {exc}") from exc


def _journal(what: str, done: subprocess.CompletedProcess, hint: str = "") -> str:
    """Log what a failed command printed, and return the short reason a reply carries."""
    printed = (done.stderr or done.stdout).strip() or f"exit {done.returncode}"
    print(f"fm-robot-agent: {what}: {printed}{'; ' + hint if hint else ''}", file=sys.stderr, flush=True)
    # A refusal by sudo is told apart from a unit that started and failed, so a
    # missing or narrower rule reads as one.
    return f"sudo refused {what}" if printed.startswith(SUDO_REFUSALS) else f"{what} failed"


def normalise(name: object) -> str | None:
    """The ``.service`` unit a caller's name means, or ``None`` when it is not one we serve."""
    if not isinstance(name, str):
        return None
    unit = name if name.endswith(".service") else f"{name}.service"
    if FM_UNIT.fullmatch(unit) or unit in CAMERA_DAEMONS:
        return unit
    return None


def refusal(action: str, unit: str) -> str | None:
    """Why ``action`` may not run on a normalised ``unit``, or ``None`` when it may."""
    if unit in CAMERA_DAEMONS and action not in CAMERA_DAEMON_ACTIONS:
        return f"{unit} is the platform's; it takes {', '.join(CAMERA_DAEMON_ACTIONS)} only"
    if unit == AGENT_UNIT and action == "stop":
        return f"{AGENT_UNIT} cannot stop itself: nothing on the fabric could start it again; restart it instead"
    return None


def active_state(unit: str) -> str:
    """One unit's ``systemctl is-active`` answer."""
    try:
        done = _run(["systemctl", "is-active", unit], STATE_TIMEOUT_S)
    except NoAnswer:
        return UNRESPONSIVE
    except AdapterError:
        return ABSENT
    return done.stdout.strip() or UNRESPONSIVE


def settled_state(unit: str) -> str:
    """``active`` once the unit has stayed active for :data:`SETTLE_S`, else the state it fell to."""
    deadline = time.monotonic() + SETTLE_S
    while True:
        state = active_state(unit)
        if state != "active" or time.monotonic() >= deadline:
            return state
        time.sleep(SETTLE_POLL_S)


def systemctl(action: str, unit: str, hint: str = "") -> str | None:
    """Run one change under sudo, returning ``None`` or the short reason it failed.

    ``hint`` says where the sudo rule comes from; it goes to the journal with
    what sudo printed.
    """
    try:
        done = _run([*PRIVILEGED_SYSTEMCTL, action, unit], SYSTEMCTL_TIMEOUT_S)
    except NoAnswer:
        return f"systemctl {action} {unit} took longer than {SYSTEMCTL_TIMEOUT_S} s"
    except AdapterError as exc:
        print(f"fm-robot-agent: {exc}", file=sys.stderr, flush=True)
        return f"systemctl {action} {unit} could not run"
    if done.returncode != 0:
        return _journal(f"systemctl {action} {unit}", done, hint)
    return None


def change(action: str, unit: str) -> tuple[bool, str]:
    """Start, stop or restart one unit, and whether it ended where it was asked to."""
    if unit == AGENT_UNIT and action == "restart":
        return _restart_agent()
    failure = systemctl(action, unit)
    if failure is not None:
        return False, failure
    if action == "stop":
        return True, f"{unit} stopped"
    done = f"{unit} {'started' if action == 'start' else 'restarted'}"
    state = settled_state(unit)
    if state == "active":
        return True, done
    if state != "inactive":
        return False, f"{unit} is {state} after {action}"
    # A oneshot without RemainAfterExit runs to completion and goes back to
    # inactive; that is a clean run, not a death. Any other type that is
    # inactive has stopped.
    try:
        shown = _properties(unit, "Type", "Result")
    except AdapterError:
        return False, f"{unit} is inactive after {action}, and whether it ran clean could not be read"
    if shown.get("Type") == "oneshot" and shown.get("Result") == "success":
        return True, f"{done}; it ran to completion"
    return False, f"{unit} is inactive after {action}"


def describe(units: list[str]) -> list[dict]:
    """``name``, ``active``, ``sub`` and ``enabled`` for each unit, in order.

    A unit systemd has never heard of answers ``not-found`` under ``enabled``
    rather than being dropped, so a status names the unit the caller asked for.
    """
    if not units:
        return []
    output = _read(["systemctl", "show", "--property=LoadState,ActiveState,SubState,UnitFileState", *units])
    blocks = [block for block in output.split("\n\n") if block.strip()]
    described = []
    for unit, block in zip(units, blocks, strict=True):
        fields = dict(line.partition("=")[::2] for line in block.splitlines() if "=" in line)
        entry = {"name": unit, **{wire: fields.get(prop, "") for prop, wire in SHOWN.items()}}
        if fields.get("LoadState") == "not-found":
            entry["enabled"] = "not-found"
        described.append(entry)
    return described


def listed() -> list[dict]:
    """Every First Motive unit this host knows, loaded or only installed, then the camera daemons it has."""
    names = set()
    for command in (["list-units", "--all"], ["list-unit-files"]):
        output = _read(["systemctl", *command, "--plain", "--no-legend", "--type=service", "fm-*"])
        names.update(line.split()[0] for line in output.splitlines() if line.split())
    fm_units = sorted(name for name in names if FM_UNIT.fullmatch(name))
    return [entry for entry in describe([*fm_units, *CAMERA_DAEMONS])
            if entry["name"] in fm_units or entry["enabled"] != "not-found"]


def _restart_agent() -> tuple[bool, str]:
    if active_state(f"{RESTART_UNIT}.timer") == "active":
        return True, f"{AGENT_UNIT} already restarts within 1 s"
    try:
        done = _run(DEFERRED_RESTART, STATE_TIMEOUT_S)
    except AdapterError as exc:
        return False, f"could not schedule the restart: {exc}"
    if done.returncode != 0:
        reason = _journal("systemd-run of the agent's restart", done)
        # The timer's name is fixed, so a restart scheduled between the check
        # above and this call fails as a name already taken.
        if "already" in (done.stderr or done.stdout):
            return True, f"{AGENT_UNIT} already restarts within 1 s"
        return False, reason
    return True, f"{AGENT_UNIT} restarts in 1 s; this reply is its last"


def _properties(unit: str, *props: str) -> dict[str, str]:
    output = _read(["systemctl", "show", f"--property={','.join(props)}", unit])
    return dict(line.partition("=")[::2] for line in output.splitlines() if "=" in line)


def _read(argv: list[str]) -> str:
    """A read-only systemctl command's output, or an :class:`AdapterError` saying why not."""
    done = _run(argv, STATE_TIMEOUT_S)
    if done.returncode != 0:
        raise AdapterError(_journal(" ".join(argv[:2]), done))
    return done.stdout
