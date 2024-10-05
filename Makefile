# Usage: make [command]
SHELL:=/bin/bash

.PHONY: build # to avoid error

build:
	catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O2" && source devel/setup.bash

clean:
	rm -r build devel logs .catkin_tools

install_deps: # install packages which are not supported by rosdep
	apt update && apt install -y \
	psmisc

setup_docker:
	docker build -t noetic_image:latest -f docker/Dockerfile . --no-cache

exec_docker:
	docker exec -it noetic_container /bin/bash

run_rocker:
	rocker --x11 --user --network host --privileged --nocleanup --volume .:/home/$(shell whoami)/mppi_swerve_drive_ros --name noetic_container noetic_image:latest

run_docker:
	@if [ "$(shell docker inspect --format='{{.State.Status}}' noetic_container)" = "running" ]; then \
		$(MAKE) exec_docker; \
	elif [ "$(shell docker inspect --format='{{.State.Status}}' noetic_container)" = "exited" ]; then \
		docker start noetic_container; \
		$(MAKE) exec_docker; \
	else \
		$(MAKE) run_rocker; \
	fi

# record rosbag (all topics)
record:
	cd $(shell pwd)/rosbag; rosbag record -a

# play rosbag
## [shell 1] make play
## [shell 2] rosbag play rosbag/xxx.bag
play:
	source devel/setup.bash && roslaunch launch/rosbag_play.launch workspace:=$(shell pwd)

# gazebo_world.launch
gazebo_world:
	source devel/setup.bash && roslaunch launch/gazebo_world.launch

# gmapping.launch
gmapping:
	source devel/setup.bash && roslaunch launch/gmapping.launch workspace:=$(shell pwd)

# navigation.launch
navigation:
	source devel/setup.bash && roslaunch launch/navigation.launch workspace:=$(shell pwd)

