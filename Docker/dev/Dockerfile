
# Use the official ROS 2 Humble image as the base image
FROM althack/ros2:humble-cuda-dev

# Set up the environment
ENV ROS_DISTRO=humble
ENV ROS_INSTALL=/opt/ros/$ROS_DISTRO/setup.bash
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

ENV CC=clang
ENV CXX=clang++

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Install additional tools and dependencies
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-pip \
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

# Create a workspace directory
RUN mkdir -p /ros2_ws/src

# Set the working directory
WORKDIR /ros2_ws

# Source the ROS 2 setup file
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/ros/.bashrc

RUN pip3 install ros2-numpy
RUN pip3 install -U scikit-learn

ENV DISPLAY=:0

# Default command (opens foxglove websocket)
CMD ["ros2", "run", "foxglove_bridge", "foxglove_bridge_launch.xml", "port:=8765"]