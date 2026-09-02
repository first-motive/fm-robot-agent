"""The service's own pieces, with no Zenoh and no session.

Only :class:`FabricWatch` is testable here, and it is the piece that matters:
the severing guard's whole meaning rests on what it counts as telemetry.
"""

from __future__ import annotations

import time

from fm_robot_agent.service import FabricWatch


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
