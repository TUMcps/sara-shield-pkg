# Docker Development Environment

This project provides a Docker-based ROS 2 Humble development environment.

## Features

- ROS 2 Humble
- Non-root user `sara`
- Repository mounted at `/workspace`
- Persistent Docker volumes for `sara-shield`
- Compatible ROS networking (`ROS_DOMAIN_ID=37`)

---

## Initial Setup

Build the Docker image:

```bash
./docker/install
```

For a clean rebuild:

```bash
./docker/install --clean
```

Build and install `sara-shield`:

```bash
./docker/build_safety_shield
```

Build the ROS workspace:

```bash
./docker/run -w /workspace/ros2_ws colcon build
```

---

## Start the Container

Open a shell inside the container:

```bash
./docker/run
```

The repository is available at:

```bash
/workspace
```

The ROS workspace is located at:

```bash
/workspace/ros2_ws
```

---

## Open a Second Terminal

If the container is already running, simply run:

```bash
./docker/run
```

The script will attach to the existing container.

---

## Build the ROS Workspace

Inside the container:

```bash
cd /workspace/ros2_ws
colcon build
source install/setup.bash
```

Or from the host:

```bash
./docker/run -w /workspace/ros2_ws colcon build
```

---

## Build / Rebuild `sara-shield`

Run this whenever:

- `external/sara-shield` changes
- you pull a new version
- you want to refresh the installation

```bash
git submodule update --init --recursive
./docker/build_safety_shield
```

Then rebuild the workspace:

```bash
./docker/run -w /workspace/ros2_ws colcon build
```

---

## Clean Rebuild of `sara-shield`

Remove the cached Docker volumes:

```bash
docker volume rm safety_demo_build_safety_shield
docker volume rm safety_demo_safety_shield_install
```

Rebuild:

```bash
./docker/build_safety_shield
./docker/run -w /workspace/ros2_ws colcon build
```

---

## Run Nodes

```bash
./docker/run
cd /workspace/ros2_ws
source install/setup.bash
ros2 run human_motion_tracker human_motion_tracker
```

In another terminal:

```bash
./docker/run
cd /workspace/ros2_ws
source install/setup.bash
ros2 topic list
```

---

## ROS Discovery Refresh

If topics or nodes are missing:

```bash
ros2 daemon stop
ros2 daemon start
```

---

## Typical Workflow

### First Time

```bash
./docker/install
./docker/build_safety_shield
./docker/run -w /workspace/ros2_ws colcon build
```

### Daily Development

```bash
./docker/run
cd /workspace/ros2_ws
source install/setup.bash
```

### After Pulling Changes

```bash
git pull
git submodule update --init --recursive
./docker/build_safety_shield
./docker/run -w /workspace/ros2_ws colcon build
```