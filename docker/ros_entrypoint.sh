#!/bin/bash
## source ros noetic
source /opt/ros/noetic/setup.bash
## source project if exists
if [ -d ~/mppi_swerve_drive_ros/devel ]; then
    source ~/mppi_swerve_drive_ros/devel/setup.bash
fi
## add commands above to ~/.bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/mppi_swerve_drive_ros/devel/setup.bash" >> ~/.bashrc
## run command
exec "$@"