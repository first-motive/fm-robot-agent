"""A minimal tRPC-over-WebSocket client for the Anvil webapp.

Recording MUST go through the webapp rather than through the ROS graph: the
webapp watches ``/recording_status`` and stops any recording it did not start
itself, so a direct ``/start_recording`` call dies within milliseconds. Going
through its API also keeps the workcell's episode database consistent, so a take
recorded from the fleet shows up in the Anvil web UI and the other way round.

The wire protocol is tRPC v11's wsLink with no transformer — plain JSON — served
at ``ws://<host>:3000/trpc``. fm-desktop already speaks it; this is the same two
call shapes in Python:

    call        a query or mutation, awaiting the single data response
    first_event start a subscription, take its FIRST data event, stop it
                (the webapp exposes its list reads as subscriptions only)

Client → server::

    {"id": n, "method": "query"|"mutation"|"subscription", "params": {"path":…, "input":…}}
    {"id": n, "method": "subscription.stop"}

Server → client::

    {"id": n, "result": {"type": "started"}}
    {"id": n, "result": {"type": "data", "data": <payload>}}
    {"id": n, "error": {"message": …}}

One call opens one connection and closes it. The agent makes a handful of calls
per operator action, never a stream, so a pooled long-lived socket would be state
to keep correct for no gain.
"""

from __future__ import annotations

import json
from typing import Any

from fm_robot_agent.protocol import AdapterError

DEFAULT_TIMEOUT_S = 10.0


class TrpcClient:
    """One webapp, reached over its tRPC WebSocket."""

    def __init__(self, url: str, timeout_s: float = DEFAULT_TIMEOUT_S) -> None:
        self.url = url
        self.timeout_s = timeout_s

    def _exchange(self, request: dict, *, subscription: bool) -> Any:
        """Send one request, return the first data payload, close.

        Imported here rather than at module scope so the module loads on a host
        where the wheel is missing and the failure names the real problem.
        """
        try:
            from websockets.sync.client import connect
        except ImportError as exc:  # pragma: no cover - a packaging fault, not a path
            raise AdapterError(f"websockets is not installed: {exc}") from exc

        try:
            with connect(self.url, open_timeout=self.timeout_s) as socket:
                socket.send(json.dumps(request, sort_keys=True))
                while True:
                    message = json.loads(socket.recv(timeout=self.timeout_s))
                    if "error" in message:
                        raise AdapterError(message["error"].get("message", "tRPC error"))
                    result = message.get("result") or {}
                    kind = result.get("type")
                    if kind == "started":
                        continue
                    if kind in ("data", None):
                        if subscription:
                            socket.send(json.dumps({"id": request["id"], "method": "subscription.stop"}))
                        return result.get("data")
        except AdapterError:
            raise
        except Exception as exc:
            raise AdapterError(f"webapp at {self.url} did not answer: {exc}") from exc

    def call(self, method: str, path: str, payload: dict | None = None) -> Any:
        """A query or mutation, awaiting its single data response."""
        params: dict[str, Any] = {"path": path}
        if payload is not None:
            params["input"] = payload
        return self._exchange({"id": 1, "method": method, "params": params}, subscription=False)

    def first_event(self, path: str, payload: dict | None = None) -> Any:
        """Start a subscription, take its first data event, stop it."""
        params: dict[str, Any] = {"path": path}
        if payload is not None:
            params["input"] = payload
        return self._exchange({"id": 1, "method": "subscription", "params": params}, subscription=True)
