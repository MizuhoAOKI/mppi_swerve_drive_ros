# rich base image
FROM osrf/ros:noetic-desktop-full

# install foundation packages
## python3-catkin-tools is for "catkin build" command
## psmisc is for "killall" command
RUN apt update && apt install -y \
    python3-catkin-tools \
    psmisc

# copy project to container temporarily for installing dependencies
COPY src /home/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash" \ 
&& rosdep update && rosdep install -y --from-paths /home/src --ignore-src --rosdistro noetic

# remove /home/src after installing dependencies
RUN rm -rf /home/src

# set entrypoint including process of sourcing ros environment
COPY docker/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# set default command
CMD ["/bin/bash"]
