# ─────────── common build args ─────────────────────────────────────────────
ARG distro=jazzy       
ARG ubuntu=24.04

# ─────────── build stage ───────────────────────────────────────────────────
FROM ros:${distro}-ros-base  AS builder
ARG distro

# tools & headers needed to build everything
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${distro}-rviz2 \
        build-essential git openssh-client cmake libeigen3-dev \
        python3-colcon-common-extensions python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# rosdep cache (already initialised on Humble / Iron / Jazzy)
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; fi && rosdep update

WORKDIR /workspace

# ─── build & install sara_shield (+ submodules) ────────────────────────────
ARG SARA_SHIELD_BRANCH=sara_shield_ros_cmake_updates
RUN --mount=type=ssh \
    mkdir -p /root/.ssh && chmod 0700 /root/.ssh && \
    ssh-keyscan -H gitlab.lrz.de >> /root/.ssh/known_hosts && \
    git clone --recurse-submodules -b ${SARA_SHIELD_BRANCH} \
        git@gitlab.lrz.de:cps-robotics/sara-shield.git /tmp/sara && \
    mkdir -p /tmp/sara/safety_shield/build && \
    cd       /tmp/sara/safety_shield/build && \
    export EIGEN3_INCLUDE_DIR=/usr/include/eigen3 && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/safety_shield \
        -DEIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR} && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/sara

# make those artefacts visible to colcon later
ENV CMAKE_PREFIX_PATH=/opt/safety_shield:${CMAKE_PREFIX_PATH}
ENV LD_LIBRARY_PATH=/opt/safety_shield/lib:${LD_LIBRARY_PATH}

# ─── build your ROS 2 workspace ────────────────────────────────────────────
COPY src/ src/
RUN . /opt/ros/${distro}/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --merge-install 

# ─────────── runtime stage ────────────────────────────────────────────────
### ─── runtime stage ─────────────────────────────────────────────
FROM ros:${distro}-ros-base
ARG distro 
# Install tini ➊ and rviz2 ➋ in a single layer
RUN apt-get update && apt-get install -y --no-install-recommends \
        tini \                             
        ros-${distro}-rviz2 \ 
        ros-${distro}-rmw-cyclonedds-cpp \             
    && rm -rf /var/lib/apt/lists/*         

# copy artefacts produced in the build stage
COPY --from=builder /workspace/install /workspace/install
COPY --from=builder /opt/safety_shield    /opt/safety_shield
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV CMAKE_PREFIX_PATH=/opt/safety_shield:${CMAKE_PREFIX_PATH}
ENV LD_LIBRARY_PATH=/opt/safety_shield/lib:${LD_LIBRARY_PATH}
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /workspace
ENTRYPOINT ["/usr/bin/tini","--","/entrypoint.sh"]
CMD ["bash"]

