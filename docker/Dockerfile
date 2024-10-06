# rich base image
FROM osrf/ros:noetic-desktop-full

# install catkin tools first
RUN apt update && apt install -y \
    python3-catkin-tools \ 
    bash-completion htop

# copy project to container temporarily for installing dependencies
COPY src /home/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash" \ 
&& rosdep update && rosdep install -y --from-paths /home/src --ignore-src --rosdistro noetic

# install dependencies
COPY Makefile /home/Makefile
RUN cd /home && make install_deps

# remove /home/src and /home/Makefile after installing dependencies
RUN rm -rf /home/src
RUN rm /home/Makefile

# set entrypoint including process of sourcing ros environment
COPY docker/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# set default command
CMD ["/bin/bash"]
