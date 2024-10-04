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

> [!NOTE]
> An academic paper related to this project has been accepted to [IROS2024](http://www.iros2024-abudhabi.org/). The source code will be open here soon.

# Setup
- [Setting up development environment on docker](docs/setup_env_on_docker.md)
- [Setting up development environment on native](docs/setup_env_on_native.md)


# Usage

## [Case 1] Launch gazebo simulator only, operating the 4wids vehicle manually with a joypad.
```bash
cd <path to your workspace>/mppi_swerve_drive_ros
source devel/setup.bash
roslaunch launch/gazebo_launcher.launch
```


> [!NOTE]
> COMING SOON...
