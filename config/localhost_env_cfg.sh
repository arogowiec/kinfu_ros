#!/usr/bin/env sh
export ROS_MASTER_URI=http://localhost:11311

. /opt/ros/kinetic/setup.sh
. ${HOME}/playground/zed/devel/setup.sh
exec "$@"
