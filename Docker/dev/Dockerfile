
# Use the official ROS 2 Humble image as the base image
FROM althack/ros2:humble-cuda-dev

ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME 

RUN sudo apt clean
RUN sudo apt install -y curl gnupg lsb-release

# Set up the environment
ENV ROS_DISTRO=humble
ENV ROS_INSTALL=/opt/ros/$ROS_DISTRO/setup.bash
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

ENV CC=clang
ENV CXX=clang++

ENV DEBIAN_FRONTEND=noninteractive

# Upgrade and add universe before installing system
RUN sudo apt upgrade

RUN apt install software-properties-common && \
    add-apt-repository universe

# Install all ROS 2 dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Install additional tools and dependencies
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-torch \
    nlohmann-json3-dev \
    libboost-all-dev \
    libeigen3-dev \
    libssl-dev \
    libwebsocketpp-dev \
    libpcl-dev \
    libvtk9-dev \
    # Install Compiler
    clang \
    # Install ROS 2 packages
    ros-humble-usb-cam \
    ros-humble-rclcpp \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-vision-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-eigen \
    ros-humble-pcl-ros \
    ros-humble-pcl-msgs \
    ros-humble-pcl-conversions \
    ros-humble-eigen3-cmake-module \
    ros-humble-robot-localization \
    ros-humble-cv-bridge \
    ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# Install all Scikit dependencies
RUN apt-get install -y --no-install-recommends \
    python3-sklearn \
    python3-sklearn-lib \
    python-sklearn-doc \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install -U \
    ros2-numpy \
    scikit-learn \
    ultralytics    

# Create a workspace directory
RUN mkdir -p /ros2_ws/src

# Set the working directory
WORKDIR /ros2_ws

# Source the ROS 2 setup file
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/ros/.bashrc

ENV DISPLAY=:0
ENV SHELL=/bin/bash

# Default command (opens foxglove websocket)
USER $USERNAME
CMD ["/bin/bash"]