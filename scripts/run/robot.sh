#!/usr/bin/env bash
#
# robot — drive a First Motive robot over the fleet fabric.
#
#   fm robot list                                    every robot answering on the fabric
#   fm robot fm-rob-01 status --json
#   fm robot fm-rob-01 up
#   fm robot fm-rob-01 mode openarm_v2_quest_teleop.yaml
#   fm robot fm-rob-01 record start --dataset grocery-sort-v1
#   fm robot fm-rob-01 stop
#
# This file is the noun; the first argument is the verb, exactly as fm-tools
# forwards it. The work lives in fm_robot_agent.client, so the CLI and the
# desktop drive the same key space with the same rules.
#
# Exit codes: 0 success, 1 the robot refused, 2 usage, 3 precondition
# (no router endpoint, or the robot did not answer).

set -euo pipefail

_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FM_ROOT="$(cd "$_here/../.." && pwd)"

exec uv run --project "$FM_ROOT" fm-robot "$@"
