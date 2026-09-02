"""The Zenoh session, and nothing else.

Deliberately thin: it reads the card, picks the adapter that card names, opens a
client-mode session against the fleet router, declares one queryable over this
robot's namespace, and hands every query to :func:`fm_robot_agent.verbs.answer`.
All the behaviour worth testing lives in :mod:`fm_robot_agent.verbs` and the
adapters, which import no Zenoh — so the suite needs neither a router nor a robot.

Client mode, not peer: every byte crosses the fleet through the router on 7447,
which is the one port the tailnet ACL opens. A robot that gossiped directly with
its peers would need a second hole for each of them.
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import threading
import time

from fm_robot_agent.anvil import KIND as ANVIL_KIND
from fm_robot_agent.anvil import AnvilAdapter
from fm_robot_agent.axol import KIND as AXOL_KIND
from fm_robot_agent.axol import AxolAdapter
from fm_robot_agent.card import CardError, RobotCard, read_card
from fm_robot_agent.env import EndpointError, router_endpoint
from fm_robot_agent.fake import FakeAdapter
from fm_robot_agent.protocol import AdapterError, RobotAdapter
from fm_robot_agent.verbs import KEY_PREFIX, answer

#: The topic the fleet watches a robot through, and therefore the one a severing
#: config change is verified against. The Anvil's crosses through
#: ``zenoh-bridge-ros2dds``; the Axol's is published by this process.
TELEMETRY_TOPIC = "joint_states"

#: Which adapter drives which card kind. The adapters land one per robot; the
#: fake is what ``--fake`` serves and what the suite drives.
ADAPTERS = {"fake": FakeAdapter, ANVIL_KIND: AnvilAdapter, AXOL_KIND: AxolAdapter}


def build_adapter(kind: str) -> RobotAdapter:
    """The adapter for a robot kind, or raise for one no adapter exists for."""
    try:
        return ADAPTERS[kind]()
    except KeyError:
        raise CardError(f"no adapter for robot kind {kind!r}") from None


def _session_config(endpoint: str):
    """A client-mode config pointed at the fleet router.

    Imported here rather than at module scope so ``--help`` works on a host
    without the zenoh wheel and the unit's failure names the real problem.
    """
    import zenoh

    config = zenoh.Config()
    config.insert_json5("mode", json.dumps("client"))
    config.insert_json5("connect/endpoints", json.dumps([endpoint]))
    return config


def _handler(adapter: RobotAdapter, namespace: str):
    """Build the queryable callback. Closes over config so Zenoh needs none of it."""

    # query is a zenoh.Query; the type is not imported at module scope on purpose.
    def handle(query) -> None:
        import zenoh

        payload = bytes(query.payload) if query.payload is not None else None
        reply = answer(
            str(query.key_expr),
            adapter,
            namespace,
            parameters=str(query.parameters or ""),
            payload=payload,
        )
        encoding = zenoh.Encoding(reply.encoding)
        if reply.ok:
            query.reply(reply.key, reply.payload, encoding=encoding)
        else:
            # An error reply, not a dropped query: a caller waiting on a verb
            # deserves the reason rather than a timeout.
            query.reply_err(reply.payload, encoding=encoding)

    return handle


class FabricWatch:
    """When this robot's telemetry was last seen on the fabric.

    The severing guard has to answer one question: did telemetry come back after
    the restart? Asking the robot's own container is the wrong side of the hop —
    a container's DDS graph is healthy whatever interface the bridge was pointed
    at, which is how a workcell pinned to `docker0` once reported success while
    the fleet received nothing.

    This watches the key the fleet actually subscribes to, from the session the
    agent already holds. It keeps a timestamp and nothing else: no payload is
    decoded, because what is being verified is that bytes arrive at all.

    Zenoh calls :meth:`sample` from its own thread. A float assignment is atomic
    under the GIL, so no lock is needed for one stamp.
    """

    def __init__(self) -> None:
        self.last_seen = 0.0

    def sample(self, _sample: object) -> None:
        self.last_seen = time.monotonic()

    def seen_since(self, since: float) -> bool:
        """Whether a sample arrived after ``since``, as the guard asks it."""
        return self.last_seen > since


#: How long to wait before reopening a telemetry stream that dropped. The Axol's
#: server restarts on its own updates, and a robot that goes quiet for a minute
#: after one is worse than a reconnect that costs nothing.
TELEMETRY_RETRY_S = 2.0


def publish_telemetry(session, adapter: RobotAdapter, namespace: str, stop: threading.Event) -> None:
    """Forward an adapter's telemetry onto the fabric until asked to stop.

    Only the Axol has a stream to forward — the Anvil's telemetry crosses through
    ``zenoh-bridge-ros2dds`` without passing through this process at all. So this
    runs only for an adapter that offers one, rather than the protocol demanding
    a stream every robot must have.
    """
    publishers: dict[str, object] = {}
    while not stop.is_set():
        try:
            for topic, payload in adapter.telemetry():
                if stop.is_set():
                    return
                if topic not in publishers:
                    publishers[topic] = session.declare_publisher(f"{namespace}/{topic}")
                publishers[topic].put(payload)
        except AdapterError as exc:
            print(f"fm-robot-agent: telemetry stopped: {exc}", file=sys.stderr, flush=True)
        # A telemetry stream that dies must never take the verb set down with it,
        # and the socket library raises its own exception family on a drop.
        except Exception as exc:  # noqa: BLE001
            print(f"fm-robot-agent: telemetry dropped: {exc}", file=sys.stderr, flush=True)
        stop.wait(TELEMETRY_RETRY_S)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Serve this robot's control verbs over Zenoh.",
    )
    parser.add_argument(
        "--fake",
        action="store_true",
        help="Drive a robot that exists only in memory, for a bench run without hardware.",
    )
    parser.add_argument(
        "--namespace",
        default="",
        help="Override the namespace derived from this host's card. Bench runs only.",
    )
    args = parser.parse_args(argv)

    try:
        if args.fake:
            card = RobotCard(name=args.namespace.replace("_", "-") or "fm-rob-00", kind="fake")
        else:
            card = read_card()
        adapter = build_adapter(card.kind)
        endpoint = router_endpoint()
    except (CardError, EndpointError) as exc:
        print(f"fm-robot-agent: {exc}", file=sys.stderr)
        return 1

    # A severing config change is journalled before it is written and cleared
    # only once telemetry has been seen. Finding one open here means the agent
    # that opened it did not live to verify it, so it is undone before this one
    # starts answering — an unverified transport change is exactly the silent
    # failure the journal exists to end.
    finish = getattr(adapter, "finish_open_change", None)
    if finish is not None:
        inherited = finish()
        if inherited is not None:
            print(f"fm-robot-agent: {inherited.message}", flush=True)

    # A namespace is a ROS name: hyphens cannot appear in one, so an override
    # typed either way lands on the same key the card would have derived.
    namespace = (args.namespace or card.namespace).replace("-", "_")
    key = f"{KEY_PREFIX}/{namespace}/*"

    import zenoh

    with zenoh.open(_session_config(endpoint)) as session:
        session.declare_queryable(key, _handler(adapter, namespace))

        # The severing guard verifies against the fabric, so it needs a view of
        # the fabric. The adapter holds no Zenoh — this hands it a question it
        # can ask, and the subscription lives here with the session.
        if hasattr(adapter, "fabric_probe"):
            watch = FabricWatch()
            session.declare_subscriber(f"{namespace}/{TELEMETRY_TOPIC}", watch.sample)
            adapter.fabric_probe = watch.seen_since
            print(f"fm-robot-agent: verifying against {namespace}/{TELEMETRY_TOPIC}", flush=True)
        print(f"fm-robot-agent: serving {key} as {card.kind} via {endpoint}", flush=True)

        stop = threading.Event()
        if hasattr(adapter, "telemetry"):
            threading.Thread(
                target=publish_telemetry,
                args=(session, adapter, namespace, stop),
                name="telemetry",
                daemon=True,
            ).start()
            print(f"fm-robot-agent: publishing {namespace}/joint_states", flush=True)
        # Nothing else to do on this thread; Zenoh runs the handler. Wait for the
        # signal systemd sends on stop rather than spinning.
        signal.sigwait({signal.SIGINT, signal.SIGTERM})
        stop.set()
    print("fm-robot-agent: stopped", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
