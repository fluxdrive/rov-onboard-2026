# ROV Project Handoff

## Overview

This is an underwater ROV system built for photogrammetry and manual piloting. It consists of two computers communicating over a local network:

- **Onboard (Raspberry Pi 5 / CM5, AlmaLinux 10, ROS 2 Lyrical via RoboStack/pixi)**: `hydromeda@192.168.1.70` — runs flight control and camera interfaces
- **Topside (x86_64 laptop, AlmaLinux 10, ROS 2 Lyrical via RoboStack/pixi)**: `ahmad@192.168.1.69` — runs joystick driver, web dashboard, and image saving

Both sides communicate via ROS 2 Lyrical Luth over CycloneDDS with unicast peer discovery (no multicast, required for Docker host networking).

Both machines run AlmaLinux 10 with ROS 2 Lyrical from RoboStack, managed by pixi (`pixi.toml` in each repo); nothing ROS-related is installed system-wide. One-time setup: `host-setup/setup-almalinux.sh` in each repo. `start.sh` runs inside the pixi environment automatically. Docker images remain as a fallback.

---

## Hardware

| Component | Details |
|-----------|---------|
| ROV frame | BlueROV2 HEAVY, VECTORED_6DOF configuration |
| Flight controller | Pixhawk1 fmuv3 running ArduSub 4.5.7 |
| Onboard computer | Raspberry Pi 5 (testing) / CM5 (ROV), ARM64, AlmaLinux 10 |
| Topside computer | AlmaLinux 10 x86_64 laptop (RTX 5070) |
| Controller | DualShock 4 (USB, read via evdev) |
| Camera | Pi Zero 2W running libcamera HTTP server at `192.168.7.2:8080` (USB gadget ethernet on Pi 5) |

---

## Repositories

| Repo | Machine | Path |
|------|---------|------|
| `aikios/rov-onboard` | Pi 5 | `~/rov_ws/` |
| `aikios/rov-topside` | Laptop | `~/rov_topside_ws/` |

---

## Architecture

### Onboard (`rov-onboard`)

**ROS2 packages:**

- **`rov_flight`** — Core flight package
  - `joy_to_mavlink.py`: Owns `/dev/ttyACM0` serial to Pixhawk. Receives `/joy` from topside, converts to MAVLink `MANUAL_CONTROL` messages. Implements PID depth hold with Åström–Hägglund relay auto-tuner. Handles arm/disarm and photogrammetry capture trigger.
  - `mavros.launch.py`: Launches MAVROS for read-only telemetry via UDP:14550 (MAVROS does **not** forward MANUAL_CONTROL — that goes direct serial).

- **`rov_photogrammetry`** — Camera interface
  - `node.py`: Polls Pi Zero HTTP server at `192.168.7.2:8080`, publishes `/photogrammetry/preview` (JPEG) and `/photogrammetry/image` (full resolution) topics.

**DDS config:** `cyclonedds_onboard.xml` — unicast peers: topside (192.168.1.69) and self (192.168.1.70).

### Topside (`rov-topside`)

**ROS2 packages:**

- **`rov_joystick`** — Controller driver
  - `joy_publisher.py`: Reads DualShock 4 via evdev (not ros2-joy — DS4 has known issues with it). Publishes `/joy` at 20 Hz.

- **`rov_dashboard`** — Web UI
  - `server.py`: HTTP server on `:8080` + WebSocket on `:9090`. Pushes ROV state at 10 Hz to browser clients.

- **`rov_photogrammetry`** — Image saving
  - `saver.py`: Subscribes to `/photogrammetry/image`, saves timestamped JPEGs locally.

**DDS config:** `cyclonedds_topside.xml` — unicast peers: onboard (192.168.1.70) and self (192.168.1.69).

---

## Running the System

### Option A: RoboStack / pixi (recommended)

`start.sh` re-runs itself inside each repo's pixi environment, so the same command works on both machines:

```bash
# On the Pi (or let rov-onboard.service start it at boot):
cd ~/rov_ws && ./start.sh [-d]            # joy_to_mavlink, mavros, photogrammetry_node

# On the laptop:
cd ~/rov_topside_ws && ./start.sh [-d]    # joy_publisher, web_dashboard, photogrammetry_saver
```

ROS CLI on either machine: `rov-shell`, then e.g. `ros2 topic list`.

### Option B: Docker (fallback)

```bash
cd ~/rov_ws && ./start-docker.sh          # Pi
cd ~/rov_topside_ws && ./start-docker.sh  # laptop
./stop-docker.sh                          # teardown on either
```

Use `stop.sh` to stop either.

---

## Building Docker Images

**Onboard (ARM64 — build on Pi):**
```bash
cd ~/rov_ws
docker build -t rov-onboard .
```

**Topside (x86_64 — build on laptop):**
```bash
cd ~/rov_topside_ws
docker build -t rov-topside .
```

Both images are two-stage: `ros:lyrical-ros-base` builds the workspace with colcon, and the runtime image is `ros:lyrical-ros-core` plus only the packages the nodes use. `./start-docker.sh --build` rebuilds after code changes.

---

## Key Design Decisions

### Why evdev instead of ros2-joy?
DualShock 4 has axis/button mapping issues with the standard ros2-joy driver. Evdev gives reliable raw access.

### Why MANUAL_CONTROL direct serial, not through MAVROS?
MAVROS does not forward `MANUAL_CONTROL` messages from ROS topics to the FC — it only relays incoming MAVLink telemetry. `joy_to_mavlink.py` owns the serial port directly and sends MAVLink frames itself.

### Why CycloneDDS unicast?
Docker with `network_mode: host` and multi-machine setups often have multicast blocked or unreliable. Explicit unicast peer lists in `cyclonedds_*.xml` give deterministic discovery.

### Why Pi Zero 2W for the camera?
The Pi Zero 2W connects via USB gadget ethernet (`192.168.7.2`), creating an isolated camera subnet. It runs a simple libcamera HTTP server — no ROS on the camera node, minimal attack surface, easy to swap out.

### Why PID depth hold on the onboard computer?
Latency matters for control loops. Keeping depth hold on the Pi (close to the FC) avoids round-trip delay through the topside.

---

## Network Layout

```
[DualShock 4] --USB--> [Laptop :ahmad@192.168.1.69]
                              |
                    ROS2/DDS (unicast)
                              |
                       [Pi 5 :hydromeda@192.168.1.70]
                              |
                     USB serial /dev/ttyACM0
                              |
                       [Pixhawk1 / ArduSub]
                              |
                         [ESCs / Thrusters]

[Pi Zero 2W] --USB gadget eth 192.168.7.2--> [Pi 5] --/photogrammetry/*--> [Laptop saver]
```

---

## Onboarding Checklist

- [ ] Install AlmaLinux 10 on the laptop and the Pi 5 / CM5
- [ ] Clone both repos onto respective machines (see repo table above)
- [ ] Run `./host-setup/setup-almalinux.sh` on both machines (add `--network --autostart` on the Pi), then log out/in
- [ ] Verify `hydromeda@192.168.1.70` and `ahmad@192.168.1.69` are reachable
- [ ] Verify DualShock 4 appears under `/dev/input/` on laptop
- [ ] Verify `/dev/ttyACM0` exists on Pi (Pixhawk connected)
- [ ] Verify Pi Zero 2W camera reachable: `curl http://192.168.7.2:8080/stream` from Pi
- [ ] Build Docker images on respective machines
- [ ] Run `./start-docker.sh` on both, open `http://192.168.1.69:8080` in browser
- [ ] Arm ROV (L1 + R1 on DS4), verify thrusters respond

---

## Known Issues / TODOs

- The web dashboard (`rov_dashboard`) shows telemetry but UI is minimal — no graphing, no depth plot
- Depth hold PID gains are hand-tuned; relay auto-tuner is implemented but not been run in a full pool test
- No automated tests — the system should be tested in a tank before any open-water dive
- Pi Zero 2W HTTP stream has no auth; treat the camera subnet as trusted-only
