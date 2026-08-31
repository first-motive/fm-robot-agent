# CLAUDE.md

Guidance for Claude Code and Codex working in this repo.

## Purpose

fm-robot-agent is the host-side agent that makes a robot a First Motive device.
It runs on the robot's own host, outside any container, and answers one verb set
over Zenoh so the fleet reaches every robot through the router on 7447 and no
other port.

## Conventions

- Commit and branch rules live in `CONTRIBUTING.md`. Follow them.
- Commits are subject-line-only: `prefix: phrase`. No body.
- Python tooling goes through `uv` — never bare `pip`, `python`, or `poetry`.
- The agent never accepts a motion command. Inbound is the fixed verb set only.

## Testing

```bash
uv run --extra dev pytest
```

## Layout

- `agent/` — the HTTP agent this repo was seeded from, pending the Zenoh port
- `src/fm_robot_agent/` — the Zenoh agent: verb router, adapters, service (steps 2-5)
- `tests/` — the suite, which needs no router and no robot
- `scripts/run/` — the verbs `fm` mounts through `fm.json`
- `systemd/` — the unit template `install.sh` renders
