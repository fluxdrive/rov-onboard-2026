# ROV Onboard

Onboard computer software for the underwater ROV. Runs on a Raspberry Pi 5 (testing) / Compute Module 5 (ROV) under AlmaLinux 10, with ROS 2 Lyrical Luth and MAVROS from RoboStack via pixi.

## Hardware

| Component | Details |
|-----------|---------|
| Computer | Raspberry Pi 5 (testing) / CM5 (ROV), AlmaLinux 10 aarch64 |
| ROS | ROS 2 Lyrical Luth + MAVROS from RoboStack (pixi); Docker on Raspberry Pi OS Lite as a fallback |
| Flight Controller | Pixhawk1 (fmuv3, 2MB flash), USB → `/dev/ttyACM0` |
| Firmware | ArduSub 4.5.7, VECTORED_6DOF frame (8 thrusters on SERVO1–8) |
| Depth Sensor | Blue Robotics Bar30 on I2C |
| Photogrammetry Camera | Pi Zero 2 W + Camera Module 3 via USB gadget (`192.168.7.2:8080`) |
| Topside Link | Ethernet tether to `192.168.1.69` |

## Quick Start

### Recommended: AlmaLinux 10 + RoboStack (pixi)

ROS 2 Lyrical and MAVROS come from [RoboStack](https://robostack.github.io) through [pixi](https://pixi.sh), pinned in `pixi.toml` / `pixi.lock` for both `linux-aarch64` (Pi 5 / CM5) and `linux-64` (laptop testing). Nothing ROS-related is installed system-wide.

```bash
git clone git@github.com:aikios/rov-onboard.git ~/rov_ws
cd ~/rov_ws
./host-setup/setup-almalinux.sh                         # run as your user, not sudo
./host-setup/setup-almalinux.sh --network --autostart   # optional: static IPs + start on boot
```

The script installs pixi, adds you to `dialout` (Pixhawk serial), disables Cockpit if present, trusts `192.168.1.0/24` in firewalld, then runs `pixi install`, `pixi run geoids` (GeographicLib geoid MAVROS needs), `pixi run build` and `pixi run check`.

- `--network` sets `eth0` to `192.168.1.70/24` (tether) and `usb0` to `192.168.7.1/24` (Pi Zero camera link). Override with `TETHER_IFACE`, `CAMERA_IFACE`, etc.
- `--autostart` installs `rov-onboard.service`, which runs `start.sh` on every boot (`journalctl -u rov-onboard -f` for logs).

Log out/in once (dialout), then:

```bash
./start.sh          # re-runs itself inside the pixi env; foreground (Ctrl+C to stop)
./start.sh -d       # daemon mode (logs in ./logs/)
./stop.sh
```

`start.sh` waits up to 20 s for `/dev/ttyACM0` (the Pixhawk enumerates a few seconds after boot; `joy_to_mavlink` only connects at startup), installs the geoid if missing, and builds the workspace on first run.

Day to day:

| Command | What it does |
|---|---|
| `pixi run build` | `colcon build --symlink-install` |
| `pixi run check` | Imports (rclpy, mavros_msgs, pymavlink), MAVROS present, geoid present, Pixhawk serial access |
| `pixi run geoids` | Downloads GeographicLib data into `./.geographiclib` |
| `rov-shell` (or `pixi shell`) | Shell with ROS sourced: `ros2 topic list`, `ros2 topic echo /rov/depth_current` |

Commit `pixi.lock` after the first successful install. `rosdep` doesn't apply inside pixi; add dependencies with `pixi add`.

### Alternative: Raspberry Pi OS Lite + Docker

```bash
sudo ./host-setup/setup-pi5.sh            # Docker, groups, log limits
./start-docker.sh                         # builds the Lyrical image on first run
./start-docker.sh --build                 # rebuild after code changes
./stop-docker.sh
```

## ROS2 Packages

### `rov_flight` — Flight control and joystick translation

**Node:** `joy_to_mavlink`

Owns the FC serial port directly and runs the full control loop:

- Subscribes `/joy` from topside, applies deadzone/noise filtering
- Sends `MANUAL_CONTROL` MAVLink messages to ArduSub at 20 Hz
- Forwards raw FC telemetry bytes to `UDP:14550` so MAVROS can read them (read-only bridge)
- **Depth hold:** Square button toggles PID depth hold; L1/R1 adjust setpoint in 0.25 m steps
- **PID auto-tuner:** Åström-Hägglund relay method — triggers via `/rov/autotune_trigger`, runs 4 oscillation cycles, applies Ziegler-Nichols gains automatically
- Arm/disarm via Options button; photogrammetry capture via Circle button
- Publishes `/rov/depth_setpoint`, `/rov/depth_current`, `/rov/depth_hold_active`, `/rov/pid_status`, `/rov/fc_heartbeat`

**Node:** `mavros` (via `ros2 launch rov_flight mavros.launch.py`)

Reads FC telemetry from `UDP:14550`, publishes standard `/mavros/*` topics consumed by the topside dashboard.

### `rov_photogrammetry` — Camera bridge

**Node:** `photogrammetry_node`

Bridges the Pi Zero camera server to ROS:

- Polls `http://192.168.7.2:8080/preview` at 2 Hz → publishes `sensor_msgs/CompressedImage` on `/photogrammetry/preview`
- On `/photogrammetry/capture` service call → fetches `http://192.168.7.2:8080/capture?quality=95` → publishes full-res JPEG on `/photogrammetry/image`

## ROS Topics Published

| Topic | Type | Description |
|-------|------|-------------|
| `/rov/fc_heartbeat` | `std_msgs/Bool` | FC connection alive |
| `/rov/depth_current` | `std_msgs/Float64` | Depth in metres (positive = deeper) |
| `/rov/depth_setpoint` | `std_msgs/Float64` | Active depth hold target |
| `/rov/depth_hold_active` | `std_msgs/Bool` | Depth hold engaged |
| `/rov/pid_status` | `std_msgs/String` | Human-readable PID/tuner state |
| `/photogrammetry/preview` | `sensor_msgs/CompressedImage` | 640×480 preview at ~2 fps |
| `/photogrammetry/image` | `sensor_msgs/CompressedImage` | Full-res capture (4608×2592) |
| `/mavros/state` | `mavros_msgs/State` | FC armed/mode/connected |
| `/mavros/vfr_hud` | `mavros_msgs/VfrHud` | Depth, heading, speed |
| `/mavros/battery` | `sensor_msgs/BatteryState` | Battery voltage |

## ROS Services

| Service | Type | Description |
|---------|------|-------------|
| `/photogrammetry/capture` | `std_srvs/Trigger` | Trigger a full-res still |

## Controller Mapping

| Button | Action |
|--------|--------|
| Left stick Y / X | Surge / Sway |
| Right stick Y / X | Heave / Yaw (manual mode) |
| Square | Toggle depth hold |
| L1 / R1 | Depth setpoint −0.25 m / +0.25 m |
| Circle | Photogrammetry capture |
| Options | Arm / Disarm |

## Network & DDS

CycloneDDS with unicast peer discovery (no multicast). Edit `cyclonedds_onboard.xml` to set peer IPs if your network changes:

```xml
<Peer address="192.168.1.69"/>  <!-- topside -->
<Peer address="192.168.1.70"/>  <!-- this Pi (self) -->
```

All containers and native nodes use `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (Lyrical's default is still Fast DDS, so this must stay set).

The Pi has several interfaces (`eth0`, `usb0`, `wlan0`, `docker0`). If DDS ever binds to the wrong one, uncomment the `<Interfaces>` block in `cyclonedds_onboard.xml` to pin it to the tether address.

## Flight Controller Setup

One-time setup (run natively, not in Docker):

```bash
# Flash ArduSub firmware
cd ~/ardupilot_fw
python3 uploader.py ardusub_fmuv3.apj --port /dev/ttyACM0

# Key ArduSub parameters
# FRAME_CONFIG = 2  (VECTORED_6DOF, 8 thrusters)
# ARMING_CHECK = 0  (disable for bench testing without thrusters)
# BRD_SAFETYOPTION = 0

# Software USB power cycle if FC stops responding
sudo sh -c "echo 0 > /sys/bus/usb/devices/2-1/authorized" && sleep 3
sudo sh -c "echo 1 > /sys/bus/usb/devices/2-1/authorized"
```

## Pi Zero Camera Server

The Pi Zero 2 W runs a lightweight libcamera HTTP server (not ROS). It connects to the Pi 5 via USB gadget ethernet at `192.168.7.2`.

| Endpoint | Description |
|----------|-------------|
| `GET /preview` | 640×480 JPEG (~2fps capable) |
| `GET /capture?quality=95` | 4608×2592 full-res JPEG |
| `GET /status` | Camera status JSON |

Setup guide: `pi-webcam-gadget-setup.md` on the Pi 5 at `~/pi-webcam-gadget-setup.md`.

## Dependencies

RoboStack environment (`pixi.toml`): `ros-lyrical-ros-base`, `ros-lyrical-rmw-cyclonedds-cpp`, `ros-lyrical-mavros`, `ros-lyrical-mavros-extras`, `ros-lyrical-mavros-msgs`, `ros-lyrical-std-srvs`, `lxml`, `colcon-common-extensions`, `setuptools>=68,<80`, and `pymavlink` from PyPI. GeographicLib geoid data via `pixi run geoids`.

Docker fallback (Dockerfile): the same ROS packages from the official `ros:lyrical` images.

## Related Repo

Topside pilot station: [`aikios/rov-topside`](https://github.com/aikios/rov-topside)
