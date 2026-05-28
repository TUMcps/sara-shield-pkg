# safety_shield and ROS2 Workspace Setup

This guide will walk you through installing the `sara_shield` library in a separate directory and then setting up and building your ROS 2 (Jazzy) workspace for the `safety_demo` package.

## 1. Install sara_shield Library

The `sara_shield` library must be built and installed into `/opt/safety_shield` (or a directory of your choosing) **outside** of your ROS workspace.

> **Choose your setup method:** You can either build natively on your machine or use the provided Docker scripts. Both paths are described below.

---

### Option A: Native Installation

1. Open a terminal.

2. Clone the repository into a separate location:
```bash
   git clone --recurse-submodules git@github.com:JulianBalletshofer/sara-shield-pacs.git
   cd sara_shield
```
3. Build and install the safety shield:
```bash
   cd ../ && mkdir build && cd build
   export EIGEN3_INCLUDE_DIR="/usr/include/eigen3/eigen-3.4.0"
   cmake .. -DCMAKE_INSTALL_PREFIX=/opt/safety_shield -DCMAKE_BUILD_TYPE=Release
   make -j$(nproc)
   sudo make install
```

> **Note:** You can change `/opt/safety_shield` to any other prefix, but you must export `CMAKE_PREFIX_PATH` accordingly in Step 3 of the ROS workspace setup.

---

### Option B: Docker Installation

The repository ships with Docker scripts that provide a fully configured ROS 2 Humble environment with `sara_shield` pre-built into a persistent volume. This is the recommended path if you want an isolated, reproducible setup.

**Prerequisites:** [Docker Engine](https://docs.docker.com/engine/install/) installed and running.

1. **Install Docker dependencies** (run once on a fresh machine):
```bash
   ./docker/install
```

2. **Build `sara_shield` into a persistent Docker volume:**
```bash
   ./docker/build_safety_shield
```
   This compiles `sara_shield` and caches the result in two named Docker volumes:
   - `safety_demo_safety_shield_install` — the install prefix (equivalent to `/opt/safety_shield`)

   These volumes persist across container restarts, so `sara_shield` does not need to be recompiled on every run.

3. **Build the ROS 2 workspace:**
```bash
   ./docker/run -w /workspace/ros2_ws colcon build
```
   `./docker/run` executes commands inside the container directly. The repository is mounted at `/workspace`. The container runs as the non-root user `sara` with `ROS_DOMAIN_ID=37` pre-configured for ROS networking.

#### Clean Rebuild of `sara_shield`

If you need to rebuild `sara_shield` from scratch, remove the cached volumes first:

```bash
docker volume rm safety_demo_safety_shield_install
```

Then rebuild:
   - `safety_demo_safety_shield_install` — the install prefix (equivalent to `/opt/safety_shield`)

```bash
./docker/build_safety_shield
./docker/run -w /workspace/ros2_ws colcon build
```

> **Note:** When using Docker, the `sara_shield` install prefix is handled automatically via the persistent volume. The `CMAKE_PREFIX_PATH` is pre-set inside the container, so no manual export is needed for the ROS workspace build.

---

## 2. Set Up Your ROS 2 (Jazzy) Workspace

Next, create a new ROS 2 workspace and clone in your `safety_demo` package.

1. Source your ROS 2 Jazzy installation (add to your `.bashrc` if needed):
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

2. Clone your `safety_demo` package:
   ```bash
   git clone -b safety_shield_PACS git@gitlab.lrz.de:jballetshofer/safety_demo.git
   cd safety_demo
   ```   
3. add prefix to the installed lib
   ```bash
   export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH #only in case robostack is used 
   export LD_LIBRARY_PATH=/opt/safety_shield/lib:$LD_LIBRARY_PATH
   export CMAKE_PREFIX_PATH=/opt/safety_shield:$CMAKE_PREFIX_PATH
   ```
4. Install ROS dependencies:
   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

5. Build the workspace with Colcon:
   ```bash
   colcon build
   ```

6. Source your overlay before running:
   ```bash
   source install/setup.bash
   ```

---

## 3. Usage

## Running the Safety Shield with RViz

```bash
ros2 launch safety_shield_node combined.launch.py robot_name:=<robot_name> sync_robot_position:=<bool> use_ik:=<bool> 
```
- robot_name  
  Choose the robot model (e.g., `panda`, `schunk`, `ur3`, `robco`).

- sync_robot_position (bool)  
  - If `true`, the robot initializes its position from the current joint states received on the `/joint_states` topic.  
  - If `false`, the robot initializes with the init_q values specified in the `safety_shield_params` configuration file.

## Running dummy human measurement data.

```bash
ros2 run human_motion_tracker human_motion_tracker
```
## Running dummy goal positions.

```bash
ros2 run simple_goal_publisher simple_goal_publisher
```

## This will send a goal pose as an example.

```bash
ros2 topic pub /goal_joint_states sensor_msgs/msg/JointState "{ 
  header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
  name: [ 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6' ],
  position: [ 0.5, -0.2, 0.1, 1.0, -0.5, 0.3 ]
}" --once

```