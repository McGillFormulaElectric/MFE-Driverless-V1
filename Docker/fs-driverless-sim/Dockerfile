FROM osrf/ros:humble-desktop

ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# System deps: Gazebo 11, ROS bridge packages, perception libs
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl wget git tmux \
    build-essential cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-auto \
    ros-humble-eigen3-cmake-module \
    gazebo \
    libgazebo-dev \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-dev \
    ros-humble-gazebo-plugins \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-ackermann-msgs \
    ros-humble-foxglove-bridge \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-slam-toolbox \
    ros-humble-pointcloud-to-laserscan \
    libpcl-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# rosdep
RUN rosdep init 2>/dev/null || true && rosdep update

# Python deps — numpy must come first to avoid binary incompatibility
RUN pip3 install --upgrade pip && \
    pip3 install "numpy<2" && \
    pip3 install "scipy>=1.13" scikit-learn icecream && \
    pip3 install --upgrade --force-reinstall pandas

# ft-fsd-path-planning (PyPI name is broken; must install from source)
RUN git clone --depth 1 https://github.com/papalotis/ft-fsd-path-planning.git /opt/ft-fsd-path-planning && \
    pip3 install /opt/ft-fsd-path-planning && \
    python3 -c "import fsd_path_planning; print('fsd_path_planning OK')"

# Shell: source ROS in every bash session inside the container
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /root/Develop

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
