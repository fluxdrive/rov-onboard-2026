#!/bin/bash
# start.sh — Launch all onboard ROV nodes natively
# Usage: ./start.sh        (foreground logs)
#        ./start.sh -d     (background/daemon mode)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/logs"
mkdir -p "$LOG_DIR"

# --- RoboStack: re-run this script inside the pixi environment ---
# (skipped if already inside it, e.g. via `pixi run start`, or if pixi isn't installed)
PIXI_BIN="$(command -v pixi 2>/dev/null || true)"
[ -z "$PIXI_BIN" ] && [ -x "$HOME/.pixi/bin/pixi" ] && PIXI_BIN="$HOME/.pixi/bin/pixi"
if [ -z "${PIXI_PROJECT_ROOT:-}" ] && [ -f "$SCRIPT_DIR/pixi.toml" ] && [ -n "$PIXI_BIN" ]; then
    exec "$PIXI_BIN" run --manifest-path "$SCRIPT_DIR/pixi.toml" "$SCRIPT_DIR/start.sh" "$@"
fi

# --- Cleanup stale DDS state ---
echo "[start] Cleaning up stale DDS shared memory..."
rm -f /dev/shm/cyclonedds_* 2>/dev/null || true

# --- Kill any existing ROV nodes ---
echo "[start] Killing stale ROV processes..."
pkill -f 'joy_to_mavlink' 2>/dev/null || true
pkill -f 'mavros_node' 2>/dev/null || true
pkill -f 'photogrammetry' 2>/dev/null || true
sleep 1

# --- Environment ---
if [ -n "${PIXI_PROJECT_ROOT:-}" ]; then
    # RoboStack/pixi: ROS 2 + MAVROS are already active in this environment
    echo "[start] Using RoboStack environment ($PIXI_PROJECT_ROOT/.pixi)"
    if [ ! -f "${GEOGRAPHICLIB_DATA:-}/geoids/egm96-5.pgm" ]; then
        echo "[start] GeographicLib geoid missing (MAVROS needs it); installing..."
        bash "$SCRIPT_DIR/scripts/install-geoids.sh" || echo "[start] WARNING: geoid install failed; MAVROS will exit"
    fi
    if [ ! -f "$SCRIPT_DIR/install/setup.bash" ]; then
        echo "[start] Workspace not built yet; building..."
        (cd "$SCRIPT_DIR" && colcon build --symlink-install)
    fi
else
    # System ROS install (e.g. Ubuntu 26.04 debs)
    ROS_DISTRO="${ROS_DISTRO:-lyrical}"
    if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
        echo "[start] No ROS found. Install pixi (curl -fsSL https://pixi.sh/install.sh | bash) and rerun,"
        echo "[start] or use ./start-docker.sh."
        exit 1
    fi
    if [ ! -f "$SCRIPT_DIR/install/setup.bash" ]; then
        echo "[start] Workspace not built. Run: source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install"
        exit 1
    fi
    source "/opt/ros/$ROS_DISTRO/setup.bash"
fi
source "$SCRIPT_DIR/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$SCRIPT_DIR/cyclonedds_onboard.xml

DAEMON=false
if [[ "$1" == "-d" ]]; then
    DAEMON=true
fi

launch_node() {
    local name="$1"
    shift
    if $DAEMON; then
        echo "[start] Launching $name (background, log: $LOG_DIR/$name.log)"
        nohup "$@" > "$LOG_DIR/$name.log" 2>&1 &
        echo $! > "$LOG_DIR/$name.pid"
    else
        echo "[start] Launching $name"
        "$@" &
    fi
}

# --- Flight controller serial port ---
# joy_to_mavlink connects to the Pixhawk only once, at startup. At boot the Pixhawk's USB
# serial can appear a few seconds after this script starts, so wait for it briefly.
FC_PORT="${FC_PORT:-/dev/ttyACM0}"
FC_WAIT_S="${FC_WAIT_S:-20}"
if [ ! -e "$FC_PORT" ]; then
    echo "[start] Waiting up to ${FC_WAIT_S}s for $FC_PORT (Pixhawk)..."
    for _ in $(seq "$FC_WAIT_S"); do
        [ -e "$FC_PORT" ] && break
        sleep 1
    done
fi
if [ ! -e "$FC_PORT" ]; then
    echo "[start] WARNING: $FC_PORT not found. joy_to_mavlink won't reach the FC until you plug it in and rerun ./start.sh"
elif [ ! -w "$FC_PORT" ]; then
    echo "[start] WARNING: no write access to $FC_PORT. Run: sudo usermod -aG dialout \$USER, then log in again"
fi

# --- 1. joy_to_mavlink (must start first — owns serial, DDS discovery order) ---
launch_node joy_to_mavlink ros2 run rov_flight joy_to_mavlink
sleep 2

# --- 2. MAVROS (reads FC telemetry from UDP:14550) ---
launch_node mavros ros2 launch rov_flight mavros.launch.py
sleep 1

# --- 3. Photogrammetry node (Pi Zero camera bridge) ---
launch_node photogrammetry ros2 run rov_photogrammetry node

echo "[start] All onboard nodes launched."

if ! $DAEMON; then
    echo "[start] Press Ctrl+C to stop all nodes."
    trap 'echo "[start] Stopping..."; kill 0; exit' SIGINT SIGTERM
    wait
fi
