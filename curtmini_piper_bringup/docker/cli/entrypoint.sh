#!/usr/bin/env bash
set -e

mkdir -p "$HOME" /diagnostics/{tf,rviz,plotjuggler,bags}
if [[ ! -e /diagnostics/rviz/robot.rviz ]]; then
  cp /opt/ws/install/share/curtmini_piper_description/rviz/view_robot.rviz \
    /diagnostics/rviz/robot.rviz
fi

exec /opt/container/ros_entrypoint.sh "$@"
