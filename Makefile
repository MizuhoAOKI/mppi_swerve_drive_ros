# Usage: make [command]

SHELL := /bin/bash
WORKSPACE := ~/mppi_swerve_drive_ros
USER = $(shell whoami)

.PHONY: build # to avoid error

build:
	catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O2"

gazebo:
	source $(WORKSPACE)/devel/setup.bash && roslaunch launch/gazebo_launcher.launch

clean:
	rm -r build devel logs .catkin_tools

setup_docker:
	docker build -t noetic_image:latest -f docker/Dockerfile . --no-cache

exec_docker:
	docker exec -it noetic_container /bin/bash

run_rocker:
	rocker --x11 --user --network host --privileged --nocleanup --volume .:/home/$(shell whoami)/mppi_swerve_drive_ros --name noetic_container noetic_image:latest

run_docker:
	@if [ "$(shell docker inspect --format='{{.State.Status}}' noetic_container)" = "running" ]; then \
		$(MAKE) dexec; \
	elif [ "$(shell docker inspect --format='{{.State.Status}}' noetic_container)" = "exited" ]; then \
		docker start noetic_container; \
		$(MAKE) exec_docker; \
	else \
		$(MAKE) run_rocker; \
	fi
