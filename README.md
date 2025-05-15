# safety_shield and ROS2 Workspace Setup

This guide will walk you through installing the `sara_shield` library in a separate directory and then setting up and building your ROS 2 (Jazzy) workspace for the `safety_demo` package.

## 1. Install **sara_shield** Library

The `sara_shield` library must be built and installed into `/opt/safety_shield` (or a directory of your choosing) **outside** of your ROS workspace.

1. Open a terminal.
2. Clone the repository into a separate location (e.g. `~/safety_shield_src`):
   ```bash
   mkdir -p ~/safety_shield_src && cd ~/safety_shield_src
   git clone --recurse-submodules -b sara_shield_ros_cmake_updates git@gitlab.lrz.de:cps-robotics/sara-shield.git
   cd sara_shield
   ```
3. add eigen
   ```bash   
   export EIGEN3_INCLUDE_DIR="/usr/include/eigen3/eigen-3.4.0"
   ```
4. Create a build directory and configure with CMake:
   ```bash
   mkdir -p build && cd build
   cmake .. -DCMAKE_INSTALL_PREFIX=/opt/safety_shield
   ```
5. Build with all available cores:
   ```bash
   make -j$(nproc)
   ```
6. Install (requires sudo):
   ```bash
   sudo make install
   ```
> **Note:** You can change `/opt/safety_shield` to any other prefix, but you must export `CMAKE_PREFIX_PATH` accordingly in step 3 of the ROS workspace setup.

---

## 2. Set Up Your ROS 2 (Jazzy) Workspace

Next, create a new ROS 2 workspace and clone in your `safety_demo` package.

1. Source your ROS 2 Jazzy installation (add to your `.bashrc` if needed):
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

2. Clone your `safety_demo` package:
   ```bash
   git clone git@gitlab.lrz.de:jballetshofer/safety_demo.git
   cd safety_demo
   ```
3. add prefix to the installed lib
   ```bash
      echo 'export LD_LIBRARY_PATH=/opt/safety_shield/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
      source ~/.bashrc
   ```
4. **Ensure CMake can find** `sara_shield` (only if you used a custom prefix):
   ```bash
   export CMAKE_PREFIX_PATH=/opt/safety_shield:$CMAKE_PREFIX_PATH
   ```

5. Install ROS dependencies:
   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

6. Build the workspace with Colcon:
   ```bash
   colcon build --symlink-install
   ```

7. Source your overlay before running:
   ```bash
   source install/setup.bash
   ```

---

## 3. Usage

### Launch the Safety Demo

```bash
ros2 launch safety_shield_node safety.launch.py
```

This will start the safety shield node.

```bash
ros2 run human_motion_tracker human_motion_tracker
```

This will start sending human measurements.

```bash
ros2 launch safety_shield_node rviz.launch.py
```

This will start rviz.

```bash
ros2 run simple_goal_publisher simple_goal_publisher
```

This will start sending goal positions.

```bash
ros2 topic pub /goal_joint_states sensor_msgs/msg/JointState "{ 
  header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
  name: [ 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6' ],
  position: [ 0.5, -0.2, 0.1, 1.0, -0.5, 0.3 ]
}" --once

```

This will send a goal pose as an example.
### Run Tests

```bash
colcon test --packages-select safety_demo
colcon test-result --verbose
```

---

## 4. Clean Up & Uninstall

If you need to remove the installed `sara_shield` library:

```bash
sudo rm -rf /opt/safety_shield
```

And to clean your ROS workspace:

```bash
cd ~/ros2_jazzy_ws
rm -rf build/ install/ log/
```

---

## 🎉 You're All Set!

Enjoy building and testing your safety functions with `sara_shield` and ROS 2 Jazzy. If you run into any issues, please file an issue on the `sara_shield` or `safety_demo` GitLab repo.

