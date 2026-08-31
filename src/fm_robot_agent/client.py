"""``fm robot`` — the CLI half of the verb set.

The agent's key space is the whole API, so the client is thin by construction: it
turns a device name into a namespace, a verb and its flags into one Zenoh query,
and the reply into either a line a person reads or the JSON a script does.

    fm robot fm-rob-01 status
    fm robot fm-rob-01 mode openarm_v2_quest_teleop.yaml
    fm robot fm-rob-01 record start --dataset grocery-sort-v1
    fm robot list

``list`` is a wildcard query over ``fm/robot/*/status``: the fabric is the
registry, so discovering robots takes no hostname, no port, and no second
service. A robot that answers is online by definition.

The client runs wherever the fleet does — a Mac on the tailnet, a rig on the LAN
— and needs only the router endpoint, the same one every other component reads.
"""

from __future__ import annotations

import argparse
import json
import re
import sys

from fm_robot_agent.env import EndpointError, router_endpoint
from fm_robot_agent.verbs import KEY_PREFIX, READ_VERBS, VERBS

#: A query waits this long for a robot to answer. Compose operations detach and
#: report through `status`, so no verb should ever need longer.
QUERY_TIMEOUT_S = 15.0

EX_USAGE = 2
EX_PRECONDITION = 3

#: A device name is a card name: `fm-<abbrev>-<nn>`. Checked here because the name
#: becomes a key expression, where `*` and `/` are not characters but syntax — a
#: name carrying either would silently widen one robot's verb into the fleet's.
DEVICE_PATTERN = re.compile(r"^fm-[a-z]+-[0-9]{2}$")

#: Control characters a reply must not carry into a terminal. The text comes off
#: the fabric, so a robot — or anything speaking as one — could otherwise repaint
#: the line an operator is reading.
CONTROL_CHARACTERS = re.compile(r"[\x00-\x08\x0b-\x1f\x7f-\x9f]")


def namespace_of(device: str) -> str:
    """The namespace a device name derives (``fm-rob-01`` → ``fm_rob_01``)."""
    return device.replace("-", "_")


def safe(text: object) -> str:
    """One line of reply text, with anything that could repaint a terminal removed."""
    return CONTROL_CHARACTERS.sub("", str(text))


def build_payload(verb: str, args: argparse.Namespace) -> dict | None:
    """The body a write verb carries, or ``None`` for one that carries none."""
    if verb == "mode":
        return {"config": args.value}
    if verb == "record":
        return {"dataset": args.dataset, "action": args.value or "start"}
    return None


def query(session, key: str, payload: dict | None = None, parameters: str = "") -> list[dict]:
    """Send one query and collect every reply, decoded."""
    import zenoh

    selector = f"{key}?{parameters}" if parameters else key
    kwargs = {"timeout": QUERY_TIMEOUT_S}
    if payload is not None:
        kwargs["payload"] = json.dumps(payload).encode()
    replies = []
    for reply in session.get(selector, **kwargs):
        # An error reply carries the refusal the agent wrote; a caller needs to
        # see it exactly as a successful one, not as an absence.
        sample = reply.ok if reply.ok is not None else reply.err
        if sample is None:
            continue
        try:
            replies.append(json.loads(bytes(sample.payload)))
        except (json.JSONDecodeError, UnicodeDecodeError):
            replies.append({"ok": False, "error": "reply was not JSON", "key": str(sample.key_expr)})
    del zenoh
    return replies


def open_session():
    """A client-mode session against this host's router."""
    import zenoh

    config = zenoh.Config()
    config.insert_json5("mode", json.dumps("client"))
    config.insert_json5("connect/endpoints", json.dumps([router_endpoint()]))
    return zenoh.open(config)


def render(replies: list[dict], as_json: bool) -> None:
    if as_json:
        print(json.dumps(replies if len(replies) != 1 else replies[0], indent=2, sort_keys=True))
        return
    for reply in replies:
        if reply.get("ok") is False:
            print(f"refused: {safe(reply.get('error') or reply.get('message') or 'no reason given')}")
            continue
        shown = {k: v for k, v in reply.items() if k != "schema_version"}
        print(safe(json.dumps(shown, sort_keys=True)))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="fm robot",
        description="Drive a First Motive robot over the fleet fabric.",
    )
    parser.add_argument("device", help="the robot's device name, or `list` to discover them")
    parser.add_argument("verb", nargs="?", choices=VERBS, help=f"one of: {', '.join(VERBS)}")
    parser.add_argument("value", nargs="?", help="mode: the config; record: start | stop")
    parser.add_argument("--dataset", default="", help="the dataset a record or episodes verb acts on")
    parser.add_argument("--json", action="store_true", dest="as_json", help="print the raw reply")
    args = parser.parse_args(argv)

    listing = args.device == "list"
    if not listing and not DEVICE_PATTERN.match(args.device):
        parser.error(f"{args.device!r} is not a device name (fm-rob-01), and only `list` queries many")
    if not listing and args.verb is None:
        parser.error("a verb is required")
    if not listing and args.verb == "mode" and not args.value:
        parser.error("mode needs a config")
    if not listing and args.verb in ("record", "episodes") and not args.dataset:
        parser.error(f"{args.verb} needs --dataset")

    try:
        session = open_session()
    except EndpointError as exc:
        print(f"fm robot: {exc}", file=sys.stderr)
        return EX_PRECONDITION

    with session:
        if listing:
            replies = query(session, f"{KEY_PREFIX}/*/status")
            render(replies, args.as_json)
            return 0

        key = f"{KEY_PREFIX}/{namespace_of(args.device)}/{args.verb}"
        parameters = f"dataset={args.dataset}" if args.verb in READ_VERBS and args.dataset else ""
        replies = query(session, key, build_payload(args.verb, args), parameters)

    if not replies:
        print(f"fm robot: {args.device} did not answer", file=sys.stderr)
        return EX_PRECONDITION
    render(replies, args.as_json)
    return 0 if all(reply.get("ok", True) for reply in replies) else 1


if __name__ == "__main__":
    raise SystemExit(main())
