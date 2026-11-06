# ROV Onboard image: ROS 2 Lyrical Luth (Ubuntu 26.04 inside the container).
# The host can be any Linux with Docker (we use Raspberry Pi OS Lite 64-bit).
#
# Two stages to keep the runtime image small:
#   build   -> ros-base (has colcon) builds the workspace
#   runtime -> ros-core (no compilers/build tools) + only the packages we use

ARG ROS_DISTRO=lyrical

# ---------------------------------------------------------------- build stage
FROM ros:${ROS_DISTRO}-ros-base AS build

WORKDIR /ros_ws
COPY src/ src/

# ament_python packages only, so no ROS deps are needed to build
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --event-handlers console_direct-

# -------------------------------------------------------------- runtime stage
FROM ros:${ROS_DISTRO}-ros-core

ENV DEBIAN_FRONTEND=noninteractive

# Only what the nodes actually import/launch:
#   rov_flight         -> mavros, mavros_extras, mavros_msgs, std_srvs, pymavlink
#   rov_photogrammetry -> sensor_msgs, std_srvs (urllib for HTTP)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras \
    ros-${ROS_DISTRO}-mavros-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    geographiclib-tools \
    wget \
    ca-certificates \
    python3-pip \
    python3-lxml \
    && rm -rf /var/lib/apt/lists/*

# GeographicLib datasets (MAVROS refuses to start without the geoid model).
# Fails the build loudly instead of producing an image that crashes at runtime.
RUN /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh || \
    (echo "ERROR: GeographicLib dataset download failed (needs internet during build)" && exit 1)

# pymavlink ships prebuilt aarch64/x86_64 wheels (lxml comes from apt above)
RUN pip3 install --break-system-packages --no-cache-dir pymavlink

WORKDIR /ros_ws
COPY --from=build /ros_ws/install /ros_ws/install

# DDS config
COPY cyclonedds_onboard.xml /ros_ws/cyclonedds.xml

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///ros_ws/cyclonedds.xml

COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh
ENTRYPOINT ["/docker-entrypoint.sh"]
