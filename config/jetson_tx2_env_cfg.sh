#!/usr/bin/env sh
export ROS_MASTER_URI=http://192.168.2.105:11311

. /opt/ros/kinetic/setup.sh
. ${HOME}/playground/zed/devel/setup.sh
exec "$@"
