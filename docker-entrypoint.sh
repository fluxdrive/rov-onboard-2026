#!/bin/bash
set +u
# ROS_DISTRO is set by the ros:<distro> base image (lyrical)
source "/opt/ros/${ROS_DISTRO:-lyrical}/setup.bash"
source /ros_ws/install/setup.bash
exec "$@"
