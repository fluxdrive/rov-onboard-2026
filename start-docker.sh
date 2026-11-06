#!/bin/bash
# ROV Onboard — start all services in Docker
# Usage: ./start-docker.sh           (builds image on first run)
#        ./start-docker.sh --build   (rebuild after code changes)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo '=== ROV Onboard (Docker) ==='

# Clean stale DDS shared memory
rm -f /dev/shm/cyclonedds_* 2>/dev/null || true

# Ensure FC serial port exists
if [ ! -e /dev/ttyACM0 ]; then
    echo '[start] WARNING: /dev/ttyACM0 not found — is the flight controller connected?'
    DEVPATH=$(find /sys/bus/usb/devices -maxdepth 1 -name '[0-9]*' 2>/dev/null | while read d; do
        [ -f "$d/idVendor" ] && grep -q 1209 "$d/idVendor" 2>/dev/null && echo "$d" && break
    done)
    if [ -n "$DEVPATH" ]; then
        echo '[start] Attempting USB power cycle...'
        sudo sh -c "echo 0 > $DEVPATH/authorized"; sleep 3
        sudo sh -c "echo 1 > $DEVPATH/authorized"; sleep 5
    fi
    [ -e /dev/ttyACM0 ] && echo '[start] FC recovered' || echo '[start] FC still not found, continuing...'
fi

# Start containers
docker compose up -d "$@"
sleep 5

# Verify
echo ''
docker compose ps --format 'table {{.Name}}\t{{.Status}}'
echo ''
docker compose logs --tail 2 joy_to_mavlink photogrammetry
