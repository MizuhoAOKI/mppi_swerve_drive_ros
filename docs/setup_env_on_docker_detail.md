# Setting up development environment on docker

## Prerequisites
- any os that supports docker (following settings was tested on ubuntu 20.04)
- [docker](https://docs.docker.com/engine/install/ubuntu/)
- [rocker](https://github.com/osrf/rocker)

## First time setup
1. clone the project repository.
    ```
    cd <path to your workspace>
    git clone https://github.com/MizuhoAOKI/mppi_swerve_drive_ros
    ```
1. build the docker image.   
   make sure to connect to the internet because dependent packages will be downloaded.
    ```
    cd <path to your workspace>/mppi_swerve_drive_ros
    docker build -t noetic_image -f docker/Dockerfile . --no-cache
    ```
1. check if the docker image is successfully built.  
    you should see `noetic_image` in the list.
    ```
    $ docker images
    REPOSITORY     TAG       IMAGE ID       CREATED        SIZE
    noetic_image   latest    da8d5c24afd6   11 hours ago   3.52GB
    ```
1. run the docker container and get into the bash inside.   
   rocker enables you to use GUI applications (ex. rviz, gazebo) and usb devices (ex. joypad) on the docker container.
    ```
    cd <path to your workspace>/mppi_swerve_drive_ros
    rocker --x11 --user --network host --privileged --nocleanup --volume .:/home/$USER/mppi_swerve_drive_ros --name noetic_container noetic_image:latest
    ```
    - shortcut of the command above:
        ```
        cd <path to your workspace>/mppi_swerve_drive_ros
        make drock
        ```
1. [inside the docker container] clean the cache.
    ```
    cd ~/mppi_swerve_drive_ros
    make clean
    ```
1. [inside the docker container] build the project.
    ```
    cd ~/mppi_swerve_drive_ros
    make build
    ```

## [After the first time setup] Use the running container
1. the command `rocker --x11 ...` above started a docker container named `noetic_container`.  
    check if noetic_container is running.   
   if it is running currently, you should see `noetic_container` in the list.  
   "STATUS" saying "Up" means the container is running now.
    ```
    $ docker ps -a
    CONTAINER ID   IMAGE          COMMAND                  CREATED         STATUS         PORTS     NAMES
    13f765669c57   655f16d731bd   "/ros_entrypoint.sh …"   4 seconds ago   Up 3 seconds             noetic_container
    ```
1. get into the bash inside the running container.
    ```
    docker exec -it noetic_container /bin/bash
    ```
    - shortcut of the command above:
        ```
        cd <path to your workspace>/mppi_swerve_drive_ros
        make dexec
        ```
1. [inside the docker container] exit the container.
    this command just exits from the bash inside the container and returns to the host terminal, but the container itself remains running.
    ```
    exit
    ```


## Other commands
-  stop the container.
    ```
    docker stop noetic_container
    ```
-  restart the container.
    ```
    docker start noetic_container
    ```
    use this command when you run `docker exec ...` but the output saids `<the target container> is not running`.
-  clean up.
    1. remove the container.
        ```
        docker rm noetic_container
        ```
    1. remove the image.
        ```
        docker rmi noetic_image:latest
        ```


## Note
On the docker environment above, you cannot access to GPU devices.
It is because this project does not heavily depend on GPU devices. 
If you need to use GPU devices, please tell us by creating an issue or a pull request.

