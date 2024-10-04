# mppi_swerve_drive_ros
**MPPI (Model Predictive Path-Integral) Controller for a Swerve Drive Robot**

<div align="center">

![image](https://github.com/user-attachments/assets/56d055e7-f3a4-4c89-940f-577b00e4f088)

[[Website]](https://mizuhoaoki.github.io/projects/iros2024)
[[PDF]](https://mizuhoaoki.github.io/media/papers/IROS2024_paper_mizuhoaoki.pdf)
[[Arxiv]](https://arxiv.org/abs/2409.08648)
[[Poster]](https://mizuhoaoki.github.io/projects/iros2024_poster.pdf)

[![ros distro: noetic](https://img.shields.io/badge/ROS-noetic-red.svg)](https://wiki.ros.org/noetic)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![Conference: IROS](https://img.shields.io/badge/Publication-IROS2024-purple.svg)](https://iros2024-abudhabi.org/)

<!-- demo movie -->
https://github.com/user-attachments/assets/0b18bb4d-c6ad-407b-919c-c8c3c219d75c

</div>


## Setup

### [Option 1] Docker environment

<details>
<summary>CLICK HERE TO EXPAND</summary>

1. Prerequisites
    - [docker](https://docs.docker.com/engine/install/ubuntu/)
    - [rocker](https://github.com/osrf/rocker)

1. Clone the project repository.
    ```
    cd <path to your workspace>
    git clone https://github.com/MizuhoAOKI/mppi_swerve_drive_ros
    ```

1. Run for the first time setup to build the docker image.
    ```
    cd <path to your workspace>/mppi_swerve_drive_ros
    make setup_docker
    ```

1. Launch the docker container and get into the bash inside.
    ```
    cd <path to your workspace>/mppi_swerve_drive_ros
    make run_docker
    ```

1. [Inside the docker container] Build the project.
    ```
    cd ~/mppi_swerve_drive_ros
    make build
    ```

</details>


### [Option 2] Native environment

<details>
<summary>CLICK HERE TO EXPAND</summary>

1. Prerequisites
    - [ubuntu 20.04](https://releases.ubuntu.com/focal/)
    - [ros noetic](https://wiki.ros.org/noetic)

1. Clone the project repository.
    ```
    cd <path to your workspace>
    git clone https://github.com/MizuhoAOKI/mppi_swerve_drive_ros
    ```

1. Install foundation packages.
    ```
    sudo apt update && sudo apt install -y python3-catkin-tools psmisc python3-rosdep
    ```
1. Initialize rosdep, update it, and install dependencies.
    ```
    cd <path to your workspace>/mppi_swerve_drive_ros
    sudo rosdep init
    rosdep update
    rosdep update && rosdep install -y --from-paths src --ignore-src --rosdistro noetic
    ```
1. Build the project.
    ```
    cd <path to your workspace>/mppi_swerve_drive_ros
    make build
    ```

</details>  


## Build

Build the project.
```
cd <path to your workspace>/mppi_swerve_drive_ros
make build
```

(Optional) Clean the cache before building the project if necessary.
```
cd <path to your workspace>/mppi_swerve_drive_ros
make clean
```


## Usage

### [Case 1] Launch gazebo simulator only, operating a 4wids vehicle manually with a joypad.
```bash
cd <path to your workspace>/mppi_swerve_drive_ros
source /opt/ros/noetic/setup.bash && source ./devel/setup.bash
roslaunch launch/gazebo_launcher.launch gazebo_world_name:=maze
```

<details>
<summary>NOTES</summary>

- Gazebo_world_name options:
    - `empty`
    - `empty_garden`
    - `cylinder_garden`
    - `maze`
- Default joystick path is `/dev/input/js0`. If you want to change the path, please edit `mppi_swerve_drive_ros/src/operation/joy_controller/config/joy.yaml`.

</details>

### [Case 2] ...

> [!NOTE]
> planner and controller nodes are coming soon...
