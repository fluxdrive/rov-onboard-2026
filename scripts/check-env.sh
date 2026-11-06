#!/bin/bash
# Quick sanity check of the onboard environment (run via `pixi run check`).
ok=true

python -c "import rclpy, mavros_msgs.msg, sensor_msgs.msg, std_srvs.srv, std_msgs.msg; from pymavlink import mavutil; import sys; print('imports OK, Python', sys.version.split()[0])" \
    || ok=false

if ros2 pkg prefix mavros >/dev/null 2>&1; then
    echo "mavros package: $(ros2 pkg prefix mavros)"
else
    echo "MISSING: mavros package"; ok=false
fi

if [[ -f "${GEOGRAPHICLIB_DATA}/geoids/egm96-5.pgm" ]]; then
    echo "geoid: ${GEOGRAPHICLIB_DATA}/geoids/egm96-5.pgm"
else
    echo "MISSING: geoid data (run: pixi run geoids)"; ok=false
fi

if [[ -e /dev/ttyACM0 ]]; then
    if [[ -r /dev/ttyACM0 && -w /dev/ttyACM0 ]]; then
        echo "flight controller: /dev/ttyACM0 (read/write OK)"
    else
        echo "flight controller: /dev/ttyACM0 present but not writable (add yourself to 'dialout' and log in again)"
        ok=false
    fi
else
    echo "flight controller: /dev/ttyACM0 not present (fine if the Pixhawk isn't plugged in)"
fi

$ok && echo "check: OK" || { echo "check: problems found"; exit 1; }
