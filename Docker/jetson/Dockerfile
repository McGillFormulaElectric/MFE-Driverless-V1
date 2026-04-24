FROM nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_79152baed139e9f4258734f3056c263a

# ── Environment ────────────────────────────────────────────────────────────────
ENV ROS_DISTRO=humble
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:0
# Orin Nano Ampere GPU — CUDA arch 87
ENV CUDAARCHS=87

# ── System packages ────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    libboost-all-dev \
    libeigen3-dev \
    libpcl-dev \
    libvtk9-dev \
    libssl-dev \
    libyaml-cpp-dev \
    nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

# ── ROS 2 packages ─────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rclcpp \
    ros-humble-rclpy \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-std-msgs \
    ros-humble-visualization-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-eigen \
    ros-humble-tf2-geometry-msgs \
    ros-humble-pcl-ros \
    ros-humble-pcl-msgs \
    ros-humble-pcl-conversions \
    ros-humble-eigen3-cmake-module \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-vision-msgs \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-usb-cam \
    ros-humble-velodyne \
    ros-humble-velodyne-driver \
    ros-humble-velodyne-pointcloud \
    ros-humble-robot-localization \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-ackermann-msgs \
    ros-humble-rviz2 \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-auto \
    ros-humble-ament-cmake-python \
    ros-humble-ament-index-cpp \
    ros-humble-rosidl-default-generators \
    && rm -rf /var/lib/apt/lists/*

# ── Python dependencies ────────────────────────────────────────────────────────
RUN pip3 install --no-cache-dir \
    ft-fsd-path-planning \
    ultralytics \
    numpy \
    scipy \
    pyrealsense2 \
    osqp \
    pyyaml \
    transforms3d

# ── OSQP C library (for future C++ MPC node) ──────────────────────────────────
RUN git clone --recurse-submodules https://github.com/osqp/osqp.git /tmp/osqp && \
    cmake -S /tmp/osqp -B /tmp/osqp/build \
        -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /tmp/osqp/build --target install -j$(nproc) && \
    rm -rf /tmp/osqp

# ── Source ROS 2 on every shell ────────────────────────────────────────────────
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

CMD ["bash"]
