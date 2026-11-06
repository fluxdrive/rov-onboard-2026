#!/bin/bash
# One-time setup for the onboard computer (Raspberry Pi 5 / CM5) on AlmaLinux 10 (aarch64).
#
# ROS 2 Lyrical + MAVROS come from RoboStack via pixi (pixi.toml in the repo root), so nothing
# ROS-related is installed system-wide. This script handles the host pieces: pixi, serial access
# to the Pixhawk, firewalld, optional static IPs, then creates the environment, downloads the
# GeographicLib geoid MAVROS needs, builds the workspace, and can install a boot-time service.
#
# Usage (from the repo root, as your normal user; it calls sudo where needed):
#   ./host-setup/setup-almalinux.sh                         # everything except network + autostart
#   ./host-setup/setup-almalinux.sh --network               # also set static IPs (eth0 tether, usb0 camera)
#   ./host-setup/setup-almalinux.sh --autostart             # also start the ROV stack on every boot
#   ./host-setup/setup-almalinux.sh --network --autostart   # both
#
# Re-running is safe.

set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# --- Network settings (only applied with --network) ---
TETHER_IFACE="${TETHER_IFACE:-eth0}"
TETHER_ADDR="${TETHER_ADDR:-192.168.1.70/24}"     # topside is 192.168.1.69
CAMERA_IFACE="${CAMERA_IFACE:-usb0}"              # Pi Zero USB gadget link; check with: nmcli device
CAMERA_ADDR="${CAMERA_ADDR:-192.168.7.1/24}"      # Pi Zero is 192.168.7.2; match ~/pi-webcam-gadget-setup.md
ROV_SUBNET="${ROV_SUBNET:-192.168.1.0/24}"

CONFIGURE_NETWORK=false
AUTOSTART=false
for arg in "$@"; do
    case "$arg" in
        --network)   CONFIGURE_NETWORK=true ;;
        --autostart) AUTOSTART=true ;;
        *) echo "Unknown option: $arg"; exit 1 ;;
    esac
done

log()  { echo -e "\n\033[1;34m[setup]\033[0m $*"; }
warn() { echo -e "\033[1;33m[setup] WARNING:\033[0m $*"; }
die()  { echo -e "\033[1;31m[setup] ERROR:\033[0m $*"; exit 1; }

[[ $EUID -eq 0 ]] && die "Run as your normal user, not root/sudo. The script uses sudo itself."

# shellcheck disable=SC1091
. /etc/os-release
if [[ "${ID:-}" != "almalinux" || "${VERSION_ID%%.*}" != "10" ]]; then
    warn "Written for AlmaLinux 10, found ${PRETTY_NAME:-unknown}. Continuing anyway."
fi
[[ "$(uname -m)" == "aarch64" ]] || warn "Expected aarch64 (Pi 5 / CM5), found $(uname -m). pixi.toml also supports linux-64 for testing."
log "Host: $PRETTY_NAME ($(uname -m)), user: $USER, repo: $REPO_DIR"

# ------------------------------------------------------------- host packages
log "Installing host tools (curl, git, tar/bzip2 for the geoid download, psmisc)..."
sudo dnf install -y curl git tar bzip2 psmisc

# --------------------------------------------------------------------- pixi
if ! command -v pixi >/dev/null 2>&1 && [[ ! -x "$HOME/.pixi/bin/pixi" ]]; then
    log "Installing pixi..."
    curl -fsSL https://pixi.sh/install.sh | bash
fi
export PATH="$HOME/.pixi/bin:$PATH"
log "pixi $(pixi --version)"

# ---------------------------------------------------- Pixhawk serial access
log "Adding $USER to dialout (read/write /dev/ttyACM0 for the Pixhawk)..."
sudo usermod -aG dialout "$USER"

# --------------------------------------------------------- unneeded services
# Cockpit (web admin on 9090) isn't needed on the ROV; disable it if the image enabled it
if systemctl is-enabled --quiet cockpit.socket 2>/dev/null; then
    log "Disabling Cockpit (not needed onboard)"
    sudo systemctl disable --now cockpit.socket
fi

# ------------------------------------------------------------------- firewalld
if systemctl is-active --quiet firewalld; then
    log "firewalld: trusting the ROV subnet $ROV_SUBNET (DDS discovery with the topside)"
    sudo firewall-cmd --permanent --zone=trusted --add-source="$ROV_SUBNET"
    sudo firewall-cmd --reload
fi

# ------------------------------------------------- static IPs (NetworkManager)
if $CONFIGURE_NETWORK; then
    command -v nmcli >/dev/null || die "nmcli not found; set static IPs manually."
    log "Tether: $TETHER_IFACE -> $TETHER_ADDR"
    sudo nmcli con delete rov-tether >/dev/null 2>&1 || true
    sudo nmcli con add type ethernet con-name rov-tether ifname "$TETHER_IFACE" \
        ipv4.method manual ipv4.addresses "$TETHER_ADDR" ipv6.method disabled \
        connection.autoconnect yes connection.autoconnect-priority 100
    log "Camera link: $CAMERA_IFACE -> $CAMERA_ADDR"
    sudo nmcli con delete rov-camera >/dev/null 2>&1 || true
    sudo nmcli con add type ethernet con-name rov-camera ifname "$CAMERA_IFACE" \
        ipv4.method manual ipv4.addresses "$CAMERA_ADDR" ipv4.never-default yes ipv6.method disabled \
        connection.autoconnect yes
    warn "If you are SSH'd in over $TETHER_IFACE with a different IP, reconnect at ${TETHER_ADDR%/*}."
fi

# ------------------------------------------------- shell convenience
if ! grep -q "# >>> rov-onboard >>>" "$HOME/.bashrc" 2>/dev/null; then
    log "Adding 'rov-shell' alias to ~/.bashrc (a shell with ROS 2 sourced)"
    cat >> "$HOME/.bashrc" <<EOF

# >>> rov-onboard >>>
export PATH="\$HOME/.pixi/bin:\$PATH"
alias rov-shell='pixi shell --manifest-path "$REPO_DIR/pixi.toml"'
# <<< rov-onboard <<<
EOF
fi

# ------------------------------- RoboStack environment, geoid data, build
cd "$REPO_DIR"
log "Creating the RoboStack environment (first run downloads a few GB; slow on an SD card)..."
pixi install
log "Downloading the GeographicLib geoid for MAVROS..."
pixi run geoids
log "Building the workspace..."
pixi run build
log "Checking the environment..."
pixi run check || warn "Some checks failed; see above (a missing /dev/ttyACM0 is fine if the Pixhawk is unplugged)."

if [[ -f pixi.lock ]] && git -C "$REPO_DIR" status --porcelain pixi.lock 2>/dev/null | grep -q .; then
    warn "pixi.lock was created/changed. Commit it so every machine gets the same versions."
fi

# ------------------------------------------------------- boot-time service
if $AUTOSTART; then
    log "Installing rov-onboard.service (starts the ROV stack on boot)"
    sudo tee /etc/systemd/system/rov-onboard.service >/dev/null <<EOF
[Unit]
Description=ROV onboard ROS 2 stack (joy_to_mavlink, MAVROS, photogrammetry)
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$REPO_DIR
Environment=PATH=$HOME/.pixi/bin:/usr/local/bin:/usr/bin:/bin
ExecStart=$REPO_DIR/start.sh
ExecStop=$REPO_DIR/stop.sh
KillMode=control-group
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
    sudo systemctl daemon-reload
    sudo systemctl enable rov-onboard.service
    echo "  Start now:   sudo systemctl start rov-onboard"
    echo "  Logs:        journalctl -u rov-onboard -f"
    echo "  Disable:     sudo systemctl disable --now rov-onboard"
fi

cat <<EOF

[setup] Done. Next steps:
  1. Log out and back in once (dialout group for the Pixhawk).
  2. cd $REPO_DIR && ./start.sh        # runs inside the RoboStack env automatically
  3. On the topside: rov-shell, then ros2 topic list   (expect /mavros/*, /rov/*, /photogrammetry/*)
  For ROS CLI tools here: rov-shell
EOF
