FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Base tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg lsb-release ca-certificates software-properties-common \
    build-essential git python3-pip clang \
    && rm -rf /var/lib/apt/lists/*

# ROS2 Humble apt repo with fresh key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list

# ROS2 + Gazebo + all stack deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-ackermann-msgs \
    ros-humble-foxglove-bridge \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-pcl-ros \
    ros-humble-pcl-msgs \
    ros-humble-pcl-conversions \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-eigen \
    ros-humble-cv-bridge \
    ros-humble-nav-msgs \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-usb-cam \
    nlohmann-json3-dev \
    libboost-all-dev \
    libeigen3-dev \
    libpcl-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Python deps
RUN pip3 install --upgrade pip && \
    pip3 install "numpy<2" ros2-numpy scikit-learn ultralytics icecream

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

WORKDIR /ros2_ws
CMD ["/bin/bash"]
