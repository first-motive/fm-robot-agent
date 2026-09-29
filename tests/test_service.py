"""The service's own pieces, with no Zenoh and no session.

:class:`FabricWatch` is the piece that matters most: the severing guard's whole
meaning rests on what it counts as telemetry. The upkeep loop is the other: it
must outlive any one failing tick.
"""

from __future__ import annotations

import threading
import time

from fm_robot_agent.protocol import AdapterError
from fm_robot_agent.service import FabricWatch, maintain_forever


class Flaky:
    """An adapter whose upkeep fails once, then works, then asks to stop."""

    def __init__(self, stop: threading.Event) -> None:
        self.stop = stop
        self.ticks = 0

    def maintain(self):
        self.ticks += 1
        if self.ticks == 1:
            raise AdapterError("webapp did not answer")
        if self.ticks == 2:
            raise RuntimeError("anything else a socket library raises")
        self.stop.set()
        return "replay buffer turned off"


def test_upkeep_survives_a_failing_tick_and_stops_when_asked(capsys):
    stop = threading.Event()
    adapter = Flaky(stop)
    maintain_forever(adapter, stop, interval_s=0.0)
    assert adapter.ticks == 3
    assert "replay buffer turned off" in capsys.readouterr().out


def test_a_watch_that_has_seen_nothing_answers_no():
    assert FabricWatch().seen_since(0.0) is False


def test_a_sample_counts_only_after_the_moment_asked_about():
    """The restart's moment is what turns a reading into a verification."""
    watch = FabricWatch()
    watch.sample(object())
    before = watch.last_seen - 1
    after = watch.last_seen + 1
    assert watch.seen_since(before) is True
    assert watch.seen_since(after) is False


def test_the_newest_sample_is_the_one_remembered():
    watch = FabricWatch()
    watch.sample(object())
    first = watch.last_seen
    time.sleep(0.01)
    watch.sample(object())
    assert watch.last_seen > first


def test_the_payload_is_never_decoded():
    """What is being verified is that bytes arrive at all."""
    watch = FabricWatch()
    watch.sample(None)
    assert watch.last_seen > 0
