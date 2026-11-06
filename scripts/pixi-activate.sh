# Sourced by pixi on `pixi run` / `pixi shell` (see [activation] in pixi.toml).
# RoboStack has already put ROS 2 Lyrical on the environment by this point.

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${PIXI_PROJECT_ROOT}/cyclonedds_onboard.xml"

# MAVROS refuses to start without the egm96-5 geoid. GeographicLib looks in
# $GEOGRAPHICLIB_DATA/geoids, so keep the data with the project (`pixi run geoids` fills it).
export GEOGRAPHICLIB_DATA="${PIXI_PROJECT_ROOT}/.geographiclib"

# Our own packages, once `pixi run build` has been run
if [ -f "${PIXI_PROJECT_ROOT}/install/setup.sh" ]; then
    . "${PIXI_PROJECT_ROOT}/install/setup.sh"
fi
