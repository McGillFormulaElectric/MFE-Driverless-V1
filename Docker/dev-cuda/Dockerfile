FROM althack/ros2:humble-cuda-dev

# Install prerequisite packages for other packages to build
RUN sudo apt clean

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

RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository universe \ 
    && rm -rf /var/lib/apt/lists/*

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
    ros-humble-navigation2 \
    ros-humble-nav2_bringup \
    ros-humble-slam-toolbox \
    ros-humble-pointcloud-to-laserscan \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get clean

RUN pip3 -V
RUN pip3 install --upgrade pip

RUN pip3 install -U \
    ros2-numpy \
    scikit-learn \ 
    ultralytics    

# Source the ROS 2 setup file
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/ros/.bashrc

ENV DISPLAY=:0

# Default command
CMD ["/bin/bash"]