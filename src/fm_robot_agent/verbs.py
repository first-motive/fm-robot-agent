"""Turn one Zenoh query into one reply — with no Zenoh in sight.

The whole verb set lands on a single queryable, so something has to route a key
to an adapter call. That routing is the part worth testing and it needs no
session, so it is a pure function here and :mod:`fm_robot_agent.service` does
nothing but carry bytes between Zenoh and this. The same split the fm-comms
episode queryable uses, for the same reason: the suite runs with no router.

    fm/robot/<ns>/status      read    robot state, one round trip
    fm/robot/<ns>/up          write   bring the stack up
    fm/robot/<ns>/down        write   take the stack down
    fm/robot/<ns>/mode        write   {"config": "..."}
    fm/robot/<ns>/record      write   {"dataset": "...", "action": "start"|"stop"}
    fm/robot/<ns>/stop        write   halt motion, vendor mechanism
    fm/robot/<ns>/episodes    read    ?dataset=<slug>

Read and write verbs are both queries. Zenoh's put/subscribe pair delivers no
reply, and every verb here has an answer the caller must see — a refused mode, a
recording that did not start. The wire contract's GET/PUT labels say whether a
verb changes the robot, not which Zenoh primitive carries it.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from urllib.parse import parse_qs

from fm_robot_agent.protocol import SCHEMA_VERSION, AdapterError, Outcome, RobotAdapter

#: Every key this agent answers sits under here. The namespace segment keeps two
#: robots on one router apart; a wildcard query over it is how the desktop
#: discovers robots without being told any hostname.
KEY_PREFIX = "fm/robot"

READ_VERBS = ("status", "episodes")
WRITE_VERBS = ("up", "down", "mode", "record", "stop")
VERBS = READ_VERBS + WRITE_VERBS

RECORD_ACTIONS = ("start", "stop")

#: Ceiling on a write verb's body. Every body the contract defines is a handful
#: of bytes, so anything past this is a caller on the fabric spending the agent's
#: memory rather than asking it for something.
MAX_BODY_BYTES = 1 << 20

#: A dataset name becomes a directory component on the robot. Matching the
#: pattern the recorder already writes is what keeps it from becoming a path.
DATASET_PATTERN = re.compile(r"^[a-z0-9]+(?:-[a-z0-9]+)*$")
DATASET_MAX_LEN = 64


@dataclass(frozen=True)
class Reply:
    """One reply to send. ``ok`` false means send it as a Zenoh error instead."""

    key: str
    payload: bytes
    encoding: str = "application/json"
    ok: bool = True


def _envelope(key: str, body: dict, ok: bool = True) -> Reply:
    """Wrap a body in the schema stamp every reply carries."""
    return Reply(
        key=key,
        payload=json.dumps({"schema_version": SCHEMA_VERSION, **body}, sort_keys=True).encode(),
        ok=ok,
    )


def _refuse(key: str, message: str) -> Reply:
    return _envelope(key, {"ok": False, "error": message}, ok=False)


def _outcome(key: str, outcome: Outcome) -> Reply:
    return _envelope(key, {"ok": outcome.ok, "message": outcome.message, **outcome.detail})


def _valid_dataset(name: str) -> bool:
    return bool(name) and len(name) <= DATASET_MAX_LEN and bool(DATASET_PATTERN.match(name))


def _body(payload: bytes | None) -> dict:
    """Decode a write verb's JSON body. An absent or oversized body is an empty one."""
    if not payload or len(payload) > MAX_BODY_BYTES:
        return {}
    try:
        decoded = json.loads(payload)
    except (json.JSONDecodeError, UnicodeDecodeError):
        return {}
    return decoded if isinstance(decoded, dict) else {}


#: The namespace segment of a discovery query. A caller that does not yet know
#: which robots exist cannot name one, so the fabric answers for all of them —
#: this is how `fm robot list` and the desktop find robots with no hostname and
#: no registry. Every other segment must match this agent's own namespace.
DISCOVERY_SEGMENT = "*"


def verb_of(key: str, namespace: str) -> str | None:
    """The verb a key names, or ``None`` when the key is not ours.

    The namespace is checked rather than trusted: one agent serves one robot, and
    answering under another robot's namespace would put two agents in a race the
    caller cannot see. A wildcard is the one exception, and only for a read:
    every agent answers such a query for itself alone, under its own key.
    """
    parts = key.split("/")
    if len(parts) != 4 or "/".join(parts[:2]) != KEY_PREFIX:
        return None
    queried_namespace, verb = parts[2], parts[3]
    if verb not in VERBS:
        return None
    if queried_namespace == namespace:
        return verb
    # A wildcard discovers; it does not command. A caller that cannot name a
    # robot has no business changing one, and `fm/robot/*/down` would otherwise
    # take the whole fleet down in a single query.
    if queried_namespace == DISCOVERY_SEGMENT and verb in READ_VERBS:
        return verb
    return None


def answer(
    key: str,
    adapter: RobotAdapter,
    namespace: str,
    *,
    parameters: str = "",
    payload: bytes | None = None,
) -> Reply:
    """Route one query to its adapter call and return the reply to send."""
    verb = verb_of(key, namespace)
    if verb is None:
        return _refuse(key, f"unknown key {key!r}")

    # Every reply carries this robot's own key, never the selector that asked.
    # A discovery query names no robot, so a reply echoing it back would leave
    # the caller unable to tell which robot answered.
    key = f"{KEY_PREFIX}/{namespace}/{verb}"

    body = _body(payload)
    try:
        if verb == "status":
            return _envelope(key, {"ok": True, "kind": adapter.kind, **adapter.status()})

        if verb == "episodes":
            dataset = (parse_qs(parameters).get("dataset") or [""])[0]
            if not _valid_dataset(dataset):
                return _refuse(key, "invalid dataset name")
            return _envelope(key, {"ok": True, "dataset": dataset, "episodes": adapter.episodes(dataset)})

        if verb == "up":
            return _outcome(key, adapter.up())

        if verb == "down":
            return _outcome(key, adapter.down())

        if verb == "stop":
            return _outcome(key, adapter.stop())

        if verb == "mode":
            config = body.get("config") or ""
            if not isinstance(config, str) or not config:
                return _refuse(key, "mode needs a config")
            return _outcome(key, adapter.set_mode(config))

        # record
        dataset = body.get("dataset") or ""
        action = body.get("action") or ""
        if not isinstance(dataset, str) or not _valid_dataset(dataset):
            return _refuse(key, "invalid dataset name")
        if action not in RECORD_ACTIONS:
            return _refuse(key, f"action must be one of {RECORD_ACTIONS}")
        return _outcome(key, adapter.record(dataset, action))
    except AdapterError as exc:
        # The robot answered with a refusal, or did not answer. Either way the
        # caller gets the reason rather than a timeout it has to guess about.
        return _refuse(key, str(exc))
