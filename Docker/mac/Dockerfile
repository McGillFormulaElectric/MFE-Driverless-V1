FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV EUFS_MASTER=/root/Develop/MFE26-eufs-sim

# Base tools & locale
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales curl wget gnupg lsb-release ca-certificates software-properties-common \
    build-essential cmake pkg-config git python3-pip clang tmux \
    python3-colcon-common-extensions python3-rosdep python3-vcstool \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 Humble stack deps + Gazebo Classic 11 ARM64 build deps (libsdformat9-dev is in Ubuntu ports)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-auto \
    ros-humble-eigen3-cmake-module \
    ros-humble-ackermann-msgs \
    ros-humble-foxglove-bridge \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-pcl-ros \
    ros-humble-pcl-msgs \
    ros-humble-pcl-conversions \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-eigen \
    ros-humble-tf2-geometry-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-vision-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-usb-cam \
    ros-humble-velodyne-pointcloud \
    ros-humble-camera-info-manager \
    ros-humble-diagnostic-updater \
    nlohmann-json3-dev \
    libboost-all-dev \
    libeigen3-dev \
    libpcl-dev \
    libyaml-cpp-dev \
    libfmt-dev \
    libtar-dev \
    libqwt-qt5-dev \
    libprotobuf-dev \
    protobuf-compiler \
    libsdformat9-dev \
    libtbb-dev \
    libogre-1.9-dev \
    libfreeimage-dev \
    libignition-math6-dev \
    libignition-transport8-dev \
    libignition-msgs5-dev \
    libignition-common3-dev \
    libignition-fuel-tools4-dev \
    libtinyxml2-dev \
    libtinyxml-dev \
    libgts-dev \
    libgdal-dev \
    libavformat-dev \
    libavcodec-dev \
    libavdevice-dev \
    libavutil-dev \
    libswscale-dev \
    libqt5widgets5 \
    qtbase5-dev \
    libbullet-dev \
    libfcl-dev \
    libassimp-dev \
    libusb-1.0-0-dev \
    libcurl4-openssl-dev \
    && rm -rf /var/lib/apt/lists/*

# Build sdformat9 (>=9.8 required by gazebo11) from source for ARM64
RUN apt-get update && apt-get install -y --no-install-recommends \
    liburdfdom-dev liburdfdom-headers-dev ruby \
    && rm -rf /var/lib/apt/lists/* && \
    git clone --depth 1 -b sdf9 https://github.com/gazebosim/sdformat.git /tmp/sdformat9 && \
    cmake -S /tmp/sdformat9 -B /tmp/sdformat9/build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    cmake --build /tmp/sdformat9/build -j2 && \
    cmake --install /tmp/sdformat9/build && \
    ldconfig && \
    rm -rf /tmp/sdformat9

# Build Gazebo Classic 11 from source for ARM64 (-j2 to stay within Docker Desktop RAM limits)
RUN git clone --depth 1 -b gazebo11 https://github.com/gazebosim/gazebo-classic.git /tmp/gazebo11 && \
    mkdir -p /tmp/gazebo11/build && cd /tmp/gazebo11/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    make -j2 && \
    make man && \
    make install && \
    ldconfig && \
    rm -rf /tmp/gazebo11

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdep init 2>/dev/null || true && rosdep update && \
    git clone --depth 1 -b ros2 https://github.com/ros-simulation/gazebo_ros_pkgs.git /tmp/gazebo_ros_pkgs && \
    rosdep install --from-paths /tmp/gazebo_ros_pkgs --ignore-src -r -y \
      --skip-keys 'gazebo libgazebo-dev gazebo11 libgazebo11-dev' 2>/dev/null || true && \
    cd /tmp/gazebo_ros_pkgs && \
    MAKEFLAGS='-j2' colcon build --merge-install --install-base /opt/ros/humble --executor sequential --cmake-args -DBUILD_TESTING=OFF && \
    rm -rf /tmp/gazebo_ros_pkgs /var/lib/apt/lists/*"

# setuptools >= 71 needs packaging >= 24.2; the system's packaging 21.3 breaks ament_python builds.
RUN pip3 install --upgrade pip && pip3 install uv "packaging>=24.2"

# Python deps — install together so uv respects the numpy<2 constraint
RUN uv pip install --system --reinstall "numpy<2" "scipy>=1.13" scikit-learn icecream pandas numba ros2-numpy

# ft-fsd-path-planning (install from source)
RUN git clone --depth 1 https://github.com/papalotis/ft-fsd-path-planning.git /opt/ft-fsd-path-planning && \
    uv pip install --system /opt/ft-fsd-path-planning && \
    python3 -c "import fsd_path_planning; print('fsd_path_planning OK')"

# CPU torch/ultralytics for ARM64 (~200 MB vs multi-GB CUDA wheels)
RUN pip3 install --retries 10 --timeout 120 --ignore-installed "sympy>=1.13" && \
    pip3 install --retries 10 --timeout 120 "numpy<2" torch torchvision --index-url https://download.pytorch.org/whl/cpu && \
    pip3 install --retries 10 --timeout 120 "numpy<2" "ultralytics==8.4.164" && \
    python3 -c "import torch, ultralytics; print('torch', torch.__version__, 'ultralytics', ultralytics.__version__)"

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

WORKDIR /root/Develop
CMD ["/bin/bash"]
