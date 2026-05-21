# Docker Development Environment

This project provides a Docker-based ROS 2 Humble development environment.

## Features

- ROS 2 Humble
- Non-root user `sara`
- Repository mounted at `/workspace`
- Persistent Docker volumes for `sara-shield`
- Compatible ROS networking (`ROS_DOMAIN_ID=37`)

---

## Typical Workflow

### First Time

```bash
./docker/install
./docker/build_safety_shield
./docker/run -w /workspace/ros2_ws colcon build
```

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