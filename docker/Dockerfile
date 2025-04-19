FROM osrf/ros:humble-desktop-full

# create the user
ARG USER=ros
ARG UUID=1000
RUN useradd -m -u $UUID -s /bin/bash $USER
RUN usermod -aG sudo $USER
RUN echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# update system basic packages
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip

# add mesa drivers for AMD gpu acceleration
RUN apt-get install -y libgl1-mesa-glx libgl1-mesa-dri

# install ros2 packages here
RUN apt-get install ros-humble-rviz2

# install python3 packages here

# environment
ENV SHELL /bin/bash
ENV ROS_DOMAIN_ID=0
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc

# set default user
USER $USER
WORKDIR /home/$USER

# entry
CMD ["bash"]
