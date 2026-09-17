#!/bin/bash
# reachy-mini-ros container entrypoint: source ROS, start the zenoh router, run the command.
set -e
source /opt/ros/${ROS_DISTRO}/setup.bash
[ -f /opt/ros/${ROS_DISTRO}/setup.bash ] && true
if ! pgrep -x rmw_zenohd >/dev/null; then
  ros2 run rmw_zenoh_cpp rmw_zenohd >/var/log/zenoh_router.log 2>&1 &
  sleep 0.5
fi
exec "$@"
