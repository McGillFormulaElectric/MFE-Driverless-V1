
# Use the official ROS 2 Humble image as the base image
FROM althack/ros2:humble-dev

# Set up the environment
ENV ROS_DISTRO=humble
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

ENV DEBIAN_FRONTEND=noninteractive

# Install additional tools and dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    libboost-all-dev \
    libeigen3-dev \
    libpcl-dev \
    libvtk9-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
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
    && rm -rf /var/lib/apt/lists/*

#TODO: add sbg driver (compile from source)

# Create a workspace directory
RUN mkdir -p /ros2_ws/src

# Set the working directory
WORKDIR /ros2_ws

# Source the ROS 2 setup file
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/ros/.bashrc

RUN pip3 install ros2-numpy
RUN pip3 install -U scikit-learn

# Default command (starts a bash shell)
CMD ["bash"]
