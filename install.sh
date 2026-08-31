#!/usr/bin/env bash
#
# install.sh — host bootstrap for fm-robot-agent.
#
# Puts the agent on a robot's own host for one role, and nothing else: the vendor
# OS owns the box, so this layers a service beside what is already there rather
# than provisioning a machine.
#
#   ./install.sh --role anvil        the Anvil workcell devbox
#   ./install.sh --role axol         the Almond Axol
#   ./install.sh --role anvil --dry-run
#
# Preconditions, checked before anything is written:
#   - this host has an identity card with role `robot` (`fm machine init`)
#   - the card's `robot` field matches the role given here
#   - /etc/fm-comms.env names a router (fm-comms' own installer writes it)
#
# The body is wrapped in main() and called on the last line, so a truncated
# `curl | bash` leaves an incomplete function that never runs.

set -euo pipefail

FM_ROLE=""
FM_DRY_RUN=0
FM_MACHINE_FILE="${FM_MACHINE_FILE:-/etc/fm/machine.json}"
FM_COMMS_ENV_FILE="${FM_COMMS_ENV_FILE:-/etc/fm-comms.env}"
UNIT_NAME="fm-robot-agent.service"

EX_USAGE=2
EX_PRECONDITION=3

# What each role's card must declare. The role is a word an operator types; the
# card's `robot` field is what the fleet derives everything else from, so the two
# are checked against each other rather than either being trusted alone.
role_kind() {
  case "$1" in
    anvil) printf '%s' "anvil-openarm-v2" ;;
    axol) printf '%s' "axol" ;;
    *) return 1 ;;
  esac
}

log() { printf '%s\n' "fm-robot-agent: $*"; }
die() { printf '%s\n' "fm-robot-agent: $1" >&2; exit "${2:-1}"; }

run() {
  if [ "$FM_DRY_RUN" = 1 ]; then
    printf '  would run: %s\n' "$*"
    return 0
  fi
  "$@"
}

usage() {
  sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
}

parse_args() {
  while [ $# -gt 0 ]; do
    case "$1" in
      --role) FM_ROLE="${2:-}"; shift 2 ;;
      --role=*) FM_ROLE="${1#--role=}"; shift ;;
      --dry-run) FM_DRY_RUN=1; shift ;;
      -h|--help) usage; exit 0 ;;
      *) die "unknown argument: $1" "$EX_USAGE" ;;
    esac
  done
  [ -n "$FM_ROLE" ] || die "--role is required (anvil | axol)" "$EX_USAGE"
  role_kind "$FM_ROLE" >/dev/null || die "unknown role: $FM_ROLE (expected anvil or axol)" "$EX_USAGE"
}

# The card is JSON, so it is read with a JSON parser. A regex over it agrees
# with the real value right up until the file holds an escaped quote or a second
# "robot" key, and this check is the one thing standing between a role an
# operator typed and a service installed on the wrong robot.
read_card_field() {
  python3 -c '
import json, sys
try:
    with open(sys.argv[1]) as handle:
        card = json.load(handle)
except (OSError, ValueError):
    sys.exit(1)
value = card.get(sys.argv[2])
print(value if isinstance(value, str) else "")
' "$1" "$2" 2>/dev/null
}

check_card() {
  local expected declared
  expected="$(role_kind "$FM_ROLE")"
  [ -f "$FM_MACHINE_FILE" ] || die "no identity card at $FM_MACHINE_FILE; run \`fm machine init\` first" "$EX_PRECONDITION"
  command -v python3 >/dev/null || die "python3 is needed to read $FM_MACHINE_FILE" "$EX_PRECONDITION"
  read_card_field "$FM_MACHINE_FILE" role >/dev/null \
    || die "$FM_MACHINE_FILE is not readable as JSON" "$EX_PRECONDITION"
  [ "$(read_card_field "$FM_MACHINE_FILE" role)" = "robot" ] \
    || die "$FM_MACHINE_FILE does not declare role robot" "$EX_PRECONDITION"
  declared="$(read_card_field "$FM_MACHINE_FILE" robot)"
  [ -n "$declared" ] || die "$FM_MACHINE_FILE declares no robot kind; re-run \`fm machine init --robot $expected\`" "$EX_PRECONDITION"
  [ "$declared" = "$expected" ] \
    || die "card says robot=$declared but --role $FM_ROLE expects $expected" "$EX_PRECONDITION"
}

check_router() {
  [ -f "$FM_COMMS_ENV_FILE" ] \
    || die "no $FM_COMMS_ENV_FILE; install fm-comms on this host first" "$EX_PRECONDITION"
  grep -q '^FM_ROUTER_ENDPOINT=' "$FM_COMMS_ENV_FILE" \
    || die "$FM_COMMS_ENV_FILE names no FM_ROUTER_ENDPOINT" "$EX_PRECONDITION"
}

install_unit() {
  local here uv user unit
  here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  uv="$(command -v uv)" || die "uv is not on PATH; install it first (https://docs.astral.sh/uv/)" "$EX_PRECONDITION"
  user="${SUDO_USER:-$USER}"
  # Substituted in bash rather than through sed: a checkout path or a username
  # holding sed's delimiter would otherwise break the render, and every value
  # here comes from the host rather than from a literal.
  # A newline in either value would append lines to the rendered unit rather
  # than fill a field in it, so neither is substituted unchecked.
  case "$user$here$uv" in
    *$'\n'*) die "the account, checkout path, or uv path holds a newline" "$EX_PRECONDITION" ;;
  esac

  local template
  template="$(cat "$here/systemd/$UNIT_NAME.in")"
  template="${template//@USER@/$user}"
  template="${template//@WORKDIR@/$here}"
  template="${template//@UV@/$uv}"
  unit="$(mktemp)"
  chmod 0600 "$unit"
  printf '%s\n' "$template" > "$unit"

  log "resolving dependencies"
  run "$uv" sync --project "$here"

  log "installing $UNIT_NAME for $user, serving from $here"
  run sudo install -m 0644 "$unit" "/etc/systemd/system/$UNIT_NAME"
  run sudo systemctl daemon-reload
  run sudo systemctl enable --now "$UNIT_NAME"
  rm -f "$unit"
}

main() {
  parse_args "$@"
  check_card
  check_router
  install_unit
  if [ "$FM_DRY_RUN" = 1 ]; then
    log "dry run: nothing was written"
    return 0
  fi
  systemctl --no-pager --lines=5 status "$UNIT_NAME" || true
  log "installed. Logs: journalctl -u $UNIT_NAME -f"
}

main "$@"
