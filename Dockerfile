ARG ROS_DISTRO=noetic
FROM osrf/ros:$ROS_DISTRO-desktop-full
ARG USERNAME=operatore
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get upgrade -y

RUN apt-get install -y python3-pip python3-venv python3-dev \
    software-properties-common \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    vim \
    tmux \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    x11-xserver-utils \
    bash-completion \
    ros-dev-tools \
    libarmadillo-dev \
    libeigen3-dev \
    libopencv-dev \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-roslint \
    ros-$ROS_DISTRO-rviz \
    && rm -rf /var/lib/apt/lists/*
    # MAVROS
    

# Install ROS 2 dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-rosinstall-generator \
    python3-vcstool \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# Setup bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /home/ws/devel/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "export DISPLAY=:1" >> /home/$USERNAME/.bashrc

ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

USER $USERNAME
CMD ["/bin/bash"]