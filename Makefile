# Usage: make [command]
SHELL:=/bin/bash
WORKSPACE=$(shell pwd)

.PHONY: build # to avoid error

build:
	export CC=clang-11 && export CXX=clang++-11 &&\
	catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O2" && source devel/setup.bash

clean:
	rm -r build devel logs .catkin_tools

install_deps: # install packages which are not supported by rosdep
	apt update && apt install -y \
	psmisc clang-11

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
		if [$? -eq 0]; then \
			$(MAKE) exec_docker; \
		else \
			docker rm noetic_container; \
			$(MAKE) run_rocker; \
		fi; \
	else \
		$(MAKE) run_rocker; \
	fi

source:
	source /opt/ros/noetic/setup.bash && source ${WORKSPACE}/devel/setup.bash

# record rosbag (all topics)
record:
	cd ${WORKSPACE}/rosbag; rosbag record -a

# play rosbag
## [shell 1] make play
## [shell 2] rosbag play rosbag/xxx.bag
play:
	$(MAKE) source && roslaunch launch/rosbag_play.launch workspace:=${WORKSPACE}

# gazebo_world.launch
gazebo_world:
	$(MAKE) source && roslaunch launch/gazebo_world.launch

# gmapping.launch
gmapping:
	$(MAKE) source && roslaunch launch/gmapping.launch workspace:=${WORKSPACE}

# navigation.launch
navigation:
	$(MAKE) source && roslaunch launch/navigation.launch workspace:=${WORKSPACE}
