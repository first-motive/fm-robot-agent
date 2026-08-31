#!/usr/bin/env python3
"""fm-anvil-agent: host-side control agent for the Anvil workcell.

Runs OUTSIDE docker (systemd, user `anvil`) so it can do the two things no
node inside the ROS graph can: rewrite the control-mode config and restart the
compose stack it would otherwise die with. It also serves the filesystem facts
the desktop app used to read over SSH (episode sizes, disk space).

Python stdlib only; no dependencies. Endpoints (JSON over HTTP, port 8770):

    GET  /health                          liveness + version
    GET  /state                           mode, configs, compose services, disk
    GET  /datasets                        dataset directory names
    GET  /episodes?dataset=<slug>         episode dirs with size + metadata.yaml
    POST /mode              {"config": "<name>.yaml", "apply": true}
    POST /stack             {"action": "up" | "down" | "recreate"}
    POST /episodes/delete   {"dataset": "<slug>", "slug": "0001"}

Security model: fixed verbs only, every input validated against a strict
pattern or the real config directory, subprocess arg lists (never a shell).
This matches the rig's existing LAN posture (rosbridge and the webapp are
equally unauthenticated); do not expose this port beyond the workcell LAN.

Compose operations run detached (their own session, output to a log file) so
an agent restart can never strand a half-recreated stack; callers poll
GET /state to watch the operation land.
"""

import json
import os
import re
import shutil
import subprocess
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

VERSION = "1.0.0"
PORT = int(os.environ.get("FM_AGENT_PORT", "8770"))
LOADER_DIR = os.environ.get(
    "FM_ANVIL_LOADER_DIR", os.path.join(os.path.expanduser("~"), "anvil-loader")
)
CONFIG_DIR = os.path.join(LOADER_DIR, "config")
ENV_CONFIG = os.path.join(LOADER_DIR, ".env.config")
RECORDINGS_DIR = os.path.join(LOADER_DIR, "data", "recordings")
COMPOSE_LOG = "/tmp/fm-anvil-agent-compose.log"

MODE_KEY = "ARMS_CONTROL_CONFIG_FILE"
DATASET_RE = re.compile(r"^[a-z0-9]+(?:-[a-z0-9]+)*$")
SLUG_RE = re.compile(r"^[0-9]{4,}$")
STACK_ACTIONS = {
    "up": ["docker", "compose", "up", "-d"],
    "down": ["docker", "compose", "down"],
    "recreate": ["docker", "compose", "up", "-d", "--force-recreate"],
}


# --- rig facts ---------------------------------------------------------------


def list_configs():
    try:
        return sorted(f for f in os.listdir(CONFIG_DIR) if f.endswith(".yaml"))
    except OSError:
        return []


def read_mode():
    """Current ARMS_CONTROL_CONFIG_FILE value; last occurrence wins."""
    value = None
    try:
        with open(ENV_CONFIG, encoding="utf-8") as f:
            for line in f:
                stripped = line.strip()
                if stripped.startswith(MODE_KEY + "="):
                    value = stripped[len(MODE_KEY) + 1 :].strip()
    except OSError:
        pass
    return value


def write_mode(name):
    """Rewrite the mode line idempotently and atomically: drop any existing
    line, append the new one, replace the file. Preserves every other key."""
    try:
        with open(ENV_CONFIG, encoding="utf-8") as f:
            lines = [l for l in f.read().splitlines() if not l.startswith(MODE_KEY + "=")]
    except OSError:
        lines = []
    lines.append(f"{MODE_KEY}={name}")
    tmp = ENV_CONFIG + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    os.replace(tmp, ENV_CONFIG)


def compose_ps():
    """Compose service rows. `--format json` emits one object per line on
    modern compose; tolerate a JSON array from older builds."""
    try:
        out = subprocess.run(
            ["docker", "compose", "ps", "--format", "json"],
            cwd=LOADER_DIR, capture_output=True, text=True, timeout=20,
        ).stdout
    except (OSError, subprocess.TimeoutExpired):
        return []
    rows = []
    text = out.strip()
    if not text:
        return []
    try:
        parsed = json.loads(text)
        rows = parsed if isinstance(parsed, list) else [parsed]
    except json.JSONDecodeError:
        for line in text.splitlines():
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return [
        {
            "name": r.get("Service") or r.get("Name") or "",
            "state": r.get("State") or "",
            "status": r.get("Status") or "",
        }
        for r in rows
        if r.get("Service") or r.get("Name")
    ]


def compose_detached(action):
    """Run a compose verb in its own session with output to the log, so the
    agent (or the stack the agent's caller rides on) restarting never strands
    it. Callers poll /state."""
    with open(COMPOSE_LOG, "ab") as log:
        subprocess.Popen(
            STACK_ACTIONS[action],
            cwd=LOADER_DIR, stdout=log, stderr=log, start_new_session=True,
        )


def disk():
    try:
        usage = shutil.disk_usage(os.path.join(LOADER_DIR, "data"))
        return {"total_kb": usage.total // 1024, "available_kb": usage.free // 1024}
    except OSError:
        return None


def dir_size_kb(path):
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(root, name))
            except OSError:
                continue
    return total // 1024


def list_datasets():
    try:
        entries = sorted(os.listdir(RECORDINGS_DIR))
    except OSError:
        return []
    return [
        e for e in entries
        if DATASET_RE.match(e) and os.path.isdir(os.path.join(RECORDINGS_DIR, e))
    ]


def list_episodes(dataset):
    base = os.path.join(RECORDINGS_DIR, dataset)
    episodes = []
    try:
        slugs = sorted(os.listdir(base))
    except OSError:
        return []
    for slug in slugs:
        path = os.path.join(base, slug)
        if not SLUG_RE.match(slug) or not os.path.isdir(path):
            continue
        metadata = ""
        try:
            with open(os.path.join(path, "metadata.yaml"), encoding="utf-8") as f:
                metadata = f.read()
        except OSError:
            pass
        episodes.append(
            {"slug": slug, "size_kb": dir_size_kb(path), "metadata_yaml": metadata}
        )
    return episodes


def delete_episode(dataset, slug):
    """Through the container: episode contents are root-owned (the ros2
    container records as root). Fixed path prefix + validated parts, arg list
    only — the rm can never escape the recordings tree."""
    result = subprocess.run(
        [
            "docker", "compose", "exec", "-T", "ros2",
            "rm", "-rf", f"/data/recordings/{dataset}/{slug}",
        ],
        cwd=LOADER_DIR, capture_output=True, text=True, timeout=60,
    )
    return result.returncode == 0, result.stderr.strip()


# --- http --------------------------------------------------------------------


class Handler(BaseHTTPRequestHandler):
    server_version = f"fm-anvil-agent/{VERSION}"

    def log_message(self, fmt, *args):  # journald gets one line per request
        print(f"{self.address_string()} {fmt % args}", flush=True)

    def reply(self, code, payload):
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def read_body(self):
        length = int(self.headers.get("Content-Length") or 0)
        if length <= 0 or length > 65536:
            return {}
        try:
            return json.loads(self.rfile.read(length))
        except json.JSONDecodeError:
            return {}

    def do_GET(self):
        url = urlparse(self.path)
        if url.path == "/health":
            self.reply(200, {"ok": True, "service": "fm-anvil-agent", "version": VERSION})
        elif url.path == "/state":
            self.reply(200, {
                "mode": read_mode(),
                "configs": list_configs(),
                "services": compose_ps(),
                "disk": disk(),
            })
        elif url.path == "/datasets":
            self.reply(200, {"datasets": list_datasets()})
        elif url.path == "/episodes":
            dataset = (parse_qs(url.query).get("dataset") or [""])[0]
            if not DATASET_RE.match(dataset) or len(dataset) > 64:
                self.reply(400, {"ok": False, "error": "invalid dataset name"})
                return
            self.reply(200, {"dataset": dataset, "episodes": list_episodes(dataset)})
        else:
            self.reply(404, {"ok": False, "error": "unknown path"})

    def do_POST(self):
        url = urlparse(self.path)
        body = self.read_body()
        if url.path == "/mode":
            config = body.get("config") or ""
            apply_now = bool(body.get("apply", True))
            if config not in list_configs():
                self.reply(400, {"ok": False, "error": "unknown config"})
                return
            write_mode(config)
            if apply_now:
                compose_detached("recreate")
            self.reply(200, {"ok": True, "mode": config, "applying": apply_now})
        elif url.path == "/stack":
            action = body.get("action") or ""
            if action not in STACK_ACTIONS:
                self.reply(400, {"ok": False, "error": "unknown action"})
                return
            compose_detached(action)
            self.reply(200, {"ok": True, "action": action, "detached": True})
        elif url.path == "/episodes/delete":
            dataset = body.get("dataset") or ""
            slug = body.get("slug") or ""
            if not DATASET_RE.match(dataset) or len(dataset) > 64 or not SLUG_RE.match(slug):
                self.reply(400, {"ok": False, "error": "invalid dataset or slug"})
                return
            ok, error = delete_episode(dataset, slug)
            self.reply(200 if ok else 500, {"ok": ok, "error": error or None})
        else:
            self.reply(404, {"ok": False, "error": "unknown path"})


def main():
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"fm-anvil-agent {VERSION} serving on :{PORT} for {LOADER_DIR}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
