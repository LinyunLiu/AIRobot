FROM osrf/ros:humble-desktop-full
LABEL author="Jacobus Burger"
LABEL supervisor="Dr Andrew Park"
LABEL descripton="A Docker image for development use in the RRU (Robotics Research Unit) Robotics Project under Dr Park"

# create the user
ARG USER=ros
ARG UUID=1000
RUN useradd -m -u $UUID -s /bin/bash $USER
RUN usermod -aG sudo $USER
RUN echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# update system basic packages
RUN apt-get update && apt-get upgrade -y

# install dev tools
RUN apt-get install -y git openssh-client neovim nano tmux
RUN apt-get install -y curl wget htop tree less
RUN apt-get install -y build-essential zip
RUN apt-get install -y bash-completion fzf ripgrep delta man

# install mesa drivers for AMD gpu acceleration
RUN apt-get install -y libgl1-mesa-glx libgl1-mesa-dri

# install ros2 packages here
RUN apt-get install -y ros-dev-tools
RUN apt-get install -y ros-humble-rviz2
RUN apt-get install -y ros-humble-navigation2 ros-humble-nav2-bringup
RUN apt-get install -y ros-humble-ros2-control ros-humble-ros2-controllers
RUN apt-get install -y ros-humble-robot-localization

# install python3 packages here
RUN apt-get install -y python3-pip python3-colcon-common-extensions

# change to user
USER $USER

# working in $HOME
WORKDIR /home/$USER/

# setup and install repository with https
# To setup with ssh, do so manually. Trying to automate that process
#   was a massive headache :(
RUN git clone https://www.github.com/CA-JunPark/odrive_ws
WORKDIR /home/$USER/odrive_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# environment
ENV SHELL=/bin/bash
ENV ROS_DOMAIN_ID=0
SHELL ["/bin/bash", "-c"]

# setup ROS2 environment source
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc
RUN echo "source /home/$USER/odrive_ws/install/setup.bash" >> /home/$USER/.bashrc

# where the user will start
CMD ["bash"]
