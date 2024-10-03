#!/bin/bash
## source ros noetic
source /opt/ros/noetic/setup.bash
## source project if exists
if [ -d ~/mppi_swerve_drive_ros/devel ]; then
    source ~/mppi_swerve_drive_ros/devel/setup.bash
fi
## run command
exec "$@"