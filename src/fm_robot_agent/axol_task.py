"""The Axol's ``task`` runs: fm-skills' task client, supervised from the host.

fm-skills runs its own container beside fm-axol's, and its task client drives
the arms through fm-axol's graph, so a run needs ``ros`` mode on and that
container up. The agent starts the client with ``docker exec`` as a background
process, keeps its last output lines, and decides from its exit and its result
line how the run ended. The agent's user is in the docker group for this
container's socket, so none of these calls take sudo.

One run at a time. The last run stays for ``status`` until the next start, so a
caller that asks after the end still sees how it went.

The router has already held the task name to :data:`fm_robot_agent.protocol.TASKS`
and object and pack to a plain name. The bus lock that keeps a start from racing
a ``stop`` is the adapter's; this module holds only its own lock over the run.
"""

from __future__ import annotations

import dataclasses
import re
import shlex
import subprocess
import sys
import threading
import time
from collections import deque

from fm_robot_agent.axol_ros import RosStack
from fm_robot_agent.protocol import AdapterError, Outcome

#: How a task runs: fm-skills' task client, inside fm-skills' own container.
DOCKER = "/usr/bin/docker"
SKILLS_CONTAINER = "fm-skills"
SKILLS_SETUP = "source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash"
#: The client's process name, which the stop matches exactly: the ``bash -lc``
#: wrapper execs into ``ros2 run``, so nothing else in the container carries it.
TASK_CLIENT = "task_client"
TASK_COMMAND = f"exec ros2 run fm_skills_task {TASK_CLIENT}"
#: Bounds on the two short docker calls: the running check and the stop signal.
SKILLS_CALL_TIMEOUT_S = 5
#: How long `stop` and `down` wait for a task to cancel its goal before they
#: take down the graph the cancel travels on. Short, because `stop` is the halt:
#: stopping the core stops the arm whether or not the cancel landed.
TASK_STOP_WAIT_S = 2
#: How much of a run's output `status` carries: lines, and characters per line.
#: The output is the client's, so a runaway line is cut rather than carried.
TASK_LOG_LINES = 40
TASK_LINE_MAX_CHARS = 500
#: The task client's last word on a run, e.g. ``result placed (code 0), pick attempts 1``.
TASK_RESULT = re.compile(r"result .*\(code (-?\d+)\).*")


@dataclasses.dataclass
class TaskRun:
    """One task run, as ``status`` reports it, from start until the next start."""

    task: str
    object: str
    pack: str
    started_at: float
    state: str = "running"
    ended_at: float | None = None
    result: str = ""
    log: deque = dataclasses.field(default_factory=lambda: deque(maxlen=TASK_LOG_LINES))
    #: Whether a stop signalled it, which is what tells a stopped run from a failed one.
    stop_requested: bool = False

    def as_dict(self) -> dict:
        return {"task": self.task, "state": self.state, "object": self.object, "pack": self.pack,
                "started_at": self.started_at, "ended_at": self.ended_at, "result": self.result,
                "log": list(self.log)}


def skills_running() -> bool:
    """Whether fm-skills' container is running. A docker that cannot answer says no."""
    try:
        done = subprocess.run([DOCKER, "inspect", "-f", "{{.State.Running}}", SKILLS_CONTAINER],
                              capture_output=True, text=True, timeout=SKILLS_CALL_TIMEOUT_S, check=False)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return done.returncode == 0 and done.stdout.strip() == "true"


def task_script(run: TaskRun) -> str:
    """The shell line fm-skills' container runs for one task.

    The router already holds object and pack to a plain name; they are quoted
    anyway, because this line is a shell's and the pattern is another module's.
    """
    params = "".join(f" -p {name}:={shlex.quote(value)}" for name, value in (("object", run.object),
                                                                             ("pack", run.pack)) if value)
    return f"{SKILLS_SETUP}; {TASK_COMMAND}" + (f" --ros-args{params}" if params else "")


def interrupt_client() -> str | None:
    """SIGINT the task client inside its container. ``None`` on success, else the reason.

    Killing the host-side ``docker exec`` would not signal the process in the
    container; on SIGINT the client cancels its goal. pkill's exit 1 is "nothing
    matched", a client that already ended, so it is not a failure.
    """
    try:
        done = subprocess.run([DOCKER, "exec", SKILLS_CONTAINER, "pkill", "-INT", "-x", TASK_CLIENT],
                              capture_output=True, text=True, timeout=SKILLS_CALL_TIMEOUT_S, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        print(f"fm-robot-agent: task stop: {exc}", file=sys.stderr, flush=True)
        return f"{SKILLS_CONTAINER} did not answer the stop"
    if done.returncode not in (0, 1):
        print(f"fm-robot-agent: task stop: {done.stderr.strip()}", file=sys.stderr, flush=True)
        return f"the stop signal to {TASK_CLIENT} failed (exit {done.returncode})"
    return None


class TaskRunner:
    """Start, follow, stop and report the one task run the Axol may have."""

    def __init__(self, ros: RosStack) -> None:
        self.ros = ros
        # Guards the run and its process, which the reader thread changes while
        # verb threads read them.
        self._lock = threading.Lock()
        self._run: TaskRun | None = None
        self._process: subprocess.Popen | None = None

    def status(self) -> Outcome:
        with self._lock:
            return Outcome(ok=True, detail={"run": self._run.as_dict() if self._run else None})

    def start(self, options: dict[str, str]) -> Outcome:
        """Start a run, or refuse with the code that says why. The caller holds the bus lock."""
        with self._lock:
            if self._process is not None:
                return Outcome(ok=False, message=f"busy: {self._run.task} is running; stop it first",
                               detail={"code": "busy"})
        if not self.ros.active():
            return Outcome(ok=False, message="a task needs mode ros; enter it first", detail={"code": "not_ros"})
        if not skills_running():
            return Outcome(ok=False, message=f"the {SKILLS_CONTAINER} container is not running",
                           detail={"code": "skills_down"})
        run = TaskRun(task=options["task"], object=options.get("object", ""), pack=options.get("pack", ""),
                      started_at=time.time())
        try:
            process = subprocess.Popen([DOCKER, "exec", SKILLS_CONTAINER, "bash", "-lc", task_script(run)],
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                       errors="replace")
        except OSError as exc:
            print(f"fm-robot-agent: task start: {exc}", file=sys.stderr, flush=True)
            raise AdapterError(f"{run.task} could not start in {SKILLS_CONTAINER}") from exc
        with self._lock:
            self._run, self._process = run, process
            started = run.as_dict()
        threading.Thread(target=self._follow, args=(process, run), daemon=True).start()
        return Outcome(ok=True, message=f"{run.task} started", detail={"run": started})

    def _follow(self, process: subprocess.Popen, run: TaskRun) -> None:
        """Keep a run's output, then settle its state once the client exits.

        The settle is in ``finally``: a reader that dies must still end the run,
        or every later start would be refused as busy.
        """
        result_code = exit_code = None
        try:
            for line in process.stdout:
                line = line.rstrip()[:TASK_LINE_MAX_CHARS]
                found = TASK_RESULT.search(line)
                with self._lock:
                    run.log.append(line)
                    if found:
                        run.result, result_code = found.group(0).strip(), found.group(1)
            exit_code = process.wait()
        except Exception as exc:  # noqa: BLE001 - any reader failure ends the run as failed
            print(f"fm-robot-agent: task output: {exc}", file=sys.stderr, flush=True)
        finally:
            with self._lock:
                run.ended_at = time.time()
                if exit_code == 0 and result_code == "0":
                    run.state = "succeeded"
                elif run.stop_requested:
                    run.state = "stopped"
                else:
                    run.state = "failed"
                self._process = None

    def stop(self) -> Outcome:
        """Signal a running task to cancel. It reads ``stopped`` once the client exits."""
        with self._lock:
            run = self._run if self._process is not None else None
            if run is None:
                return Outcome(ok=True, message="no task running", detail={"run": None})
            run.stop_requested = True
        failure = interrupt_client()
        if failure is not None:
            raise AdapterError(failure)
        with self._lock:
            return Outcome(ok=True, message=f"{run.task} stopping", detail={"run": run.as_dict()})

    def end(self) -> None:
        """Stop a running task and wait for it, so its goal is cancelled while its graph is still up.

        A stop that fails or a client that outlasts the wait is logged, not
        raised: leaving ``ros`` mode is the halt, and it must not wait on this.
        """
        with self._lock:
            process = self._process
        if process is None:
            return
        try:
            self.stop()
            process.wait(timeout=TASK_STOP_WAIT_S)
        except (AdapterError, subprocess.TimeoutExpired) as exc:
            print(f"fm-robot-agent: task did not end before mode ros stopped: {exc}", file=sys.stderr, flush=True)
