# Setting up development environment on docker

## Prerequisites
- [ubuntu 20.04](https://releases.ubuntu.com/focal/)
- [ros noetic](https://wiki.ros.org/noetic)

## Procedure
1. clone the project repository.
    ```
    cd <path to your workspace>
    git clone https://github.com/MizuhoAOKI/mppi_swerve_drive_ros
    ```
1. install foundation packages.
    ```
    sudo apt update && sudo apt install -y python3-catkin-tools psmisc python3-rosdep
    ```
1. initialize rosdep, update it, and install dependencies.
    ```
    cd <path to your workspace>/mppi_swerve_drive_ros
    sudo rosdep init
    rosdep update
    rosdep update && rosdep install -y --from-paths src --ignore-src --rosdistro noetic
    ```
1. clean the cache.
    ```
    cd <path to your workspace>/mppi_swerve_drive_ros
    make clean
    ```
1. build the project.
    ```
    cd <path to your workspace>/mppi_swerve_drive_ros
    make build
    ```

## Note
On the docker environment above, you cannot access to GPU devices.
