ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive
ENV HOME=/home/sara
SHELL ["/bin/bash", "-c"]

# =======================================================================================
# User setup
# =======================================================================================
RUN apt-get update && apt-get install -y sudo && \
    useradd --uid 1000 --create-home --shell /bin/bash sara && \
    echo 'sara:sara123' | chpasswd && \
    adduser sara sudo && \
    chown -R sara:sara /home/sara && \
    rm -rf /var/lib/apt/lists/*

# =======================================================================================
# Base dependencies
# =======================================================================================
RUN apt-get update && apt-get install -y \
    git \
    curl \
    gnupg \
    lsb-release \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    cmake \
    wget \
    gdb \
    libfmt-dev \
    libeigen3-dev \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-trajectory-msgs \
    ros-${ROS_DISTRO}-visualization-msgs \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-ament-index-cpp \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-demo-nodes-cpp \
    ros-${ROS_DISTRO}-demo-nodes-py \
    && rm -rf /var/lib/apt/lists/*

# =======================================================================================
# Eigen 3.4.0 — exact version required by sara-shield
# =======================================================================================
RUN mkdir -p /usr/include/eigen3 && \
    wget -q https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz \
        -O /tmp/eigen.tar.gz && \
    tar -xzf /tmp/eigen.tar.gz -C /usr/include/eigen3/ && \
    rm /tmp/eigen.tar.gz

# =======================================================================================
# Environment
# =======================================================================================
ENV EIGEN3_INCLUDE_DIR=/usr/include/eigen3/eigen-3.4.0
ENV LD_LIBRARY_PATH=/opt/safety_shield/lib
ENV CMAKE_PREFIX_PATH=/opt/safety_shield
ENV ROS_DOMAIN_ID=37

# Persistent Docker volumes will be mounted here by docker/run.
RUN mkdir -p /opt/safety_shield /docker_build/safety_shield && \
    chmod -R 777 /opt/safety_shield /docker_build && \
    chown -R sara:sara /opt/safety_shield /docker_build

# =======================================================================================
# rosdep
# =======================================================================================
RUN rosdep init 2>/dev/null || true && rosdep update || true

# =======================================================================================
# Shell setup
# =======================================================================================
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/sara/.bashrc && \
    echo "export EIGEN3_INCLUDE_DIR=/usr/include/eigen3/eigen-3.4.0" >> /home/sara/.bashrc && \
    echo "export LD_LIBRARY_PATH=/opt/safety_shield/lib:\$LD_LIBRARY_PATH" >> /home/sara/.bashrc && \
    echo "export CMAKE_PREFIX_PATH=/opt/safety_shield:\$CMAKE_PREFIX_PATH" >> /home/sara/.bashrc && \
    echo "export ROS_DOMAIN_ID=37" >> /home/sara/.bashrc && \
    echo '[[ -f /workspace/ros2_ws/install/setup.bash ]] && source /workspace/ros2_ws/install/setup.bash' >> /home/sara/.bashrc && \
    chown sara:sara /home/sara/.bashrc

WORKDIR /workspace

RUN mkdir -p /home/sara/.ros/log && \
    chown -R sara:sara /home/sara/.ros
USER sara

CMD ["bash", "-l"]