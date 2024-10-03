# Setting up development environment on docker

## Prerequisites
- any os that supports docker
- [docker](https://docs.docker.com/engine/install/ubuntu/)
- [rocker](https://github.com/osrf/rocker)

## Procedure
1. clone the project repository
    ```
    cd /path/to/your/workspace
    git clone https://github.com/MizuhoAOKI/mppi_swerve_drive_ros
    ```
1. build the docker image
    ```
    cd /path/to/mppi_swerve_drive_ros
    docker build -t noetic_image -f docker/Dockerfile . --no-cache
    ```
1. run the docker container and get into the bash inside. 
   rocker enables you to use GUI applications (ex. rviz, gazebo) and usb devices (ex. joypad) on the docker container.
    ```
    cd /path/to/mppi_swerve_drive_ros
    rocker --x11 --user --network host --privileged --nocleanup --volume .:/home/$USER/mppi_swerve_drive_ros --name noetic_container noetic_image:latest
    ```
1. [inside the docker container] clean the cache
    ```
    cd ~/mppi_swerve_drive_ros
    make clean
    ```
1. [inside the docker container] build the project
    ```
    cd ~/mppi_swerve_drive_ros
    make build
    ```

## Note
On the docker environment above, you cannot access to GPU devices.

