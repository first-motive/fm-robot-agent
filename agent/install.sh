#!/usr/bin/env bash
# Install fm-anvil-agent as a systemd service. Idempotent; run on the rig:
#   sudo bash ~/anvil-loader/agent/install.sh
set -euo pipefail

if [ "$EUID" -ne 0 ]; then
  printf '%s\n' "This installer needs root. Re-running with sudo..."
  exec sudo "$0" "$@"
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
UNIT_SRC="${SCRIPT_DIR}/fm-anvil-agent.service"
UNIT_DST="/etc/systemd/system/fm-anvil-agent.service"

install -m 0644 "${UNIT_SRC}" "${UNIT_DST}"
systemctl daemon-reload
systemctl enable --now fm-anvil-agent.service

# Advertise on mDNS (same _fm-rig._tcp type the First Motive rigs use) so the
# desktop app's Settings discovers this rig. avahi-daemon watches this
# directory and picks the file up without a restart.
if [ -d /etc/avahi/services ]; then
  HOST_SHORT="$(hostname -s)"
  cat > /etc/avahi/services/fm-anvil-agent.service <<EOF
<?xml version="1.0" standalone='no'?>
<!DOCTYPE service-group SYSTEM "avahi-service.dtd">
<service-group>
  <name replace-wildcards="yes">%h anvil</name>
  <service>
    <type>_fm-rig._tcp</type>
    <port>8770</port>
    <txt-record>role=anvil</txt-record>
    <txt-record>host=${HOST_SHORT}.local</txt-record>
    <txt-record>port=8770</txt-record>
  </service>
</service-group>
EOF
  printf '%s\n' "mDNS advert installed (_fm-rig._tcp, role=anvil)."
else
  printf '%s\n' "Warning: avahi not present; the rig will not be discoverable. Install avahi-daemon and re-run."
fi
sleep 1
systemctl --no-pager --lines=5 status fm-anvil-agent.service || true

if curl -sf --max-time 3 http://localhost:8770/health >/dev/null; then
  printf '%s\n' "fm-anvil-agent is up on :8770."
else
  printf '%s\n' "Warning: agent did not answer on :8770 yet; check journalctl -u fm-anvil-agent."
  exit 1
fi
