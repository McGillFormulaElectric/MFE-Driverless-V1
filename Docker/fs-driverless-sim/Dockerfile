FROM osrf/ros:humble-desktop

ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV EUFS_MASTER=/root/Develop/MFE26-eufs-sim

# Avoids locale warnings in bash
RUN apt-get update && apt-get install -y --no-install-recommends locales \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

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
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-slam-toolbox \
    ros-humble-pointcloud-to-laserscan \
    libpcl-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# CUDA 12.8 is the first toolkit with sm_120 (RTX 50-series). cuda-pcl's aarch64 libs are not used;
# the driver comes from the host (run with --gpus all). Without a GPU the node falls back to CPU PCL.
RUN apt-get update && apt-get install -y --no-install-recommends wget ca-certificates && \
    wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && rm cuda-keyring_1.1-1_all.deb && \
    apt-get update && apt-get install -y --no-install-recommends cuda-toolkit-12-8 && \
    rm -rf /var/lib/apt/lists/*
ENV PATH=/usr/local/cuda/bin:${PATH}

RUN rosdep init 2>/dev/null || true && rosdep update

# setuptools >= 71 needs packaging >= 24.2; the system's packaging 21.3 breaks ament_python builds.
RUN pip3 install --upgrade pip && pip3 install uv "packaging>=24.2"

# One uv call so the numpy<2 constraint is respected.
# numba: path planner speed (pure-Python fallback is ~10x slower).
RUN uv pip install --system "numpy<2" "scipy>=1.13" scikit-learn icecream pandas numba

# PyPI name is broken; install from source.
RUN git clone --depth 1 https://github.com/papalotis/ft-fsd-path-planning.git /opt/ft-fsd-path-planning && \
    uv pip install --system /opt/ft-fsd-path-planning && \
    python3 -c "import fsd_path_planning; print('fsd_path_planning OK')"

# gtsam 4.2: x86_64 wheel only. Jetson builds lib/gtsam (pinned 4.2.0 submodule) instead.
RUN pip3 install --retries 10 --timeout 120 "gtsam==4.2" && \
    python3 -c "import gtsam; print('gtsam OK')"

# Without torch/ultralytics the node dies on import (perception then runs LiDAR-only).
# CPU torch wheels: ~250 MB vs ~4.5 GB for CUDA, and YOLO11n on CPU keeps up with the sim camera.
# For GPU on RTX 50xx use the cu128 index (https://download.pytorch.org/whl/cu128), the first Blackwell CUDA build.
# numpy<2 repeated in each command so pip's resolver never upgrades it.
# sympy: the base image's distutils-installed 1.9 cannot be uninstalled; --ignore-installed overlays it.
RUN pip3 install --retries 10 --timeout 120 --ignore-installed "sympy>=1.13" && \
    pip3 install --retries 10 --timeout 120 "numpy<2" torch torchvision --index-url https://download.pytorch.org/whl/cpu && \
    pip3 install --retries 10 --timeout 120 "numpy<2" ultralytics && \
    python3 -c "import torch, ultralytics; print('torch', torch.__version__, 'ultralytics', ultralytics.__version__)"

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /root/Develop

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
