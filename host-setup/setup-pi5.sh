#!/bin/bash
# One-time host setup for the onboard Pi 5 on Raspberry Pi OS Lite (64-bit, Debian 13 "trixie").
#
# The host stays minimal: no ROS, no desktop. ROS 2 Lyrical runs inside Docker
# (Ubuntu 26.04 userland in the container), which is the only officially
# supported binary path for Lyrical on a Pi.
#
# Usage:
#   sudo ./host-setup/setup-pi5.sh                     # Docker, groups, log limits, trim services
#   sudo ./host-setup/setup-pi5.sh --network           # also set static IPs (see variables below)
#
# Re-running is safe.

set -euo pipefail

# --- Network settings (only applied with --network) ---
TETHER_IFACE="${TETHER_IFACE:-eth0}"
TETHER_ADDR="${TETHER_ADDR:-192.168.1.70/24}"     # topside is 192.168.1.69
CAMERA_IFACE="${CAMERA_IFACE:-usb0}"              # Pi Zero USB gadget link; check with: ip -br link
CAMERA_ADDR="${CAMERA_ADDR:-192.168.7.1/24}"      # Pi Zero is 192.168.7.2; match ~/pi-webcam-gadget-setup.md

CONFIGURE_NETWORK=false
[[ "${1:-}" == "--network" ]] && CONFIGURE_NETWORK=true

if [[ $EUID -ne 0 ]]; then echo "Run with sudo."; exit 1; fi
TARGET_USER="${SUDO_USER:-$(logname 2>/dev/null || echo root)}"

. /etc/os-release
if [[ "$(uname -m)" != "aarch64" ]]; then
    echo "WARNING: expected aarch64 (64-bit Raspberry Pi OS), got $(uname -m). The ROS image is arm64-only on Pi."
fi
echo "[setup] Host: $PRETTY_NAME ($(uname -m)), user: $TARGET_USER"

# --- Base packages ---
apt-get update
apt-get install -y --no-install-recommends ca-certificates curl git

# --- Docker Engine (Debian repo; Raspberry Pi OS 64-bit uses the Debian packages) ---
if ! command -v docker >/dev/null; then
    echo "[setup] Installing Docker Engine..."
    install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
    chmod a+r /etc/apt/keyrings/docker.asc
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian ${VERSION_CODENAME} stable" \
        > /etc/apt/sources.list.d/docker.list
    apt-get update
    apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
fi

# Cap container logs so a chatty node can't fill the SD card
if [[ ! -f /etc/docker/daemon.json ]]; then
    cat > /etc/docker/daemon.json <<'EOF'
{
  "log-driver": "json-file",
  "log-opts": { "max-size": "10m", "max-file": "3" }
}
EOF
fi
systemctl enable --now docker
systemctl restart docker

# docker without sudo; dialout for flashing the Pixhawk natively (uploader.py)
usermod -aG docker,dialout "$TARGET_USER"

# --- Trim services the ROV doesn't use ---
for svc in bluetooth hciuart ModemManager; do
    if systemctl cat "$svc.service" >/dev/null 2>&1; then
        systemctl disable --now "$svc" 2>/dev/null || true
        echo "[setup] Disabled $svc"
    fi
done

# --- Optional: static IPs via NetworkManager (Raspberry Pi OS default) ---
if $CONFIGURE_NETWORK; then
    if ! command -v nmcli >/dev/null; then
        echo "[setup] nmcli not found; set static IPs manually."
    else
        echo "[setup] Tether: $TETHER_IFACE -> $TETHER_ADDR"
        nmcli con delete rov-tether >/dev/null 2>&1 || true
        nmcli con add type ethernet con-name rov-tether ifname "$TETHER_IFACE" \
            ipv4.method manual ipv4.addresses "$TETHER_ADDR" ipv6.method disabled \
            connection.autoconnect yes connection.autoconnect-priority 100
        echo "[setup] Camera link: $CAMERA_IFACE -> $CAMERA_ADDR"
        nmcli con delete rov-camera >/dev/null 2>&1 || true
        nmcli con add type ethernet con-name rov-camera ifname "$CAMERA_IFACE" \
            ipv4.method manual ipv4.addresses "$CAMERA_ADDR" ipv4.never-default yes ipv6.method disabled \
            connection.autoconnect yes
        echo "[setup] Connections created. They activate when the cables are present (or reboot)."
        echo "[setup] If you are SSH'd in over $TETHER_IFACE with a different IP, you will need to reconnect at the new address."
    fi
fi

cat <<EOF

[setup] Done. Next steps:
  1. Log out and back in (or reboot) so '$TARGET_USER' picks up the docker group.
  2. cd ~/rov_ws && ./start-docker.sh      # first run builds the Lyrical image (needs internet)
  3. Containers use restart: unless-stopped, so after that the ROV stack starts on every boot.

  Debug from the host (no native ROS needed):
     docker compose exec mavros ros2 topic list
EOF
