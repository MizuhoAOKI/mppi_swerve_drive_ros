# Usage: make [command]

SHELL := /bin/bash
WORKSPACE := ~/mppi_swerve_drive_ros

.PHONY: build # to avoid error

build:
	catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O2"

gazebo:
	source $(WORKSPACE)/devel/setup.bash && roslaunch launch/gazebo_launcher.launch

clean:
	rm -r build devel logs .catkin_tools
