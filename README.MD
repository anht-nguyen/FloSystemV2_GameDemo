# FloSystemV2_GameDemo

A ROS-based system for controlling and simulating the FLO V2 humanoid robot, including motor control, vision, and integration with simulation/game environments.

---

## Table of Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Docker Setup](#docker-setup)
- [Building and Running with Docker](#building-and-running-with-docker)
- [Manual (Non-Docker) Build](#manual-non-docker-build)
- [Usage](#usage)
- [License](#license)

---

## Overview

This repository contains all core packages for the FLO V2 robot demo, including:

- `flo_core`: Main control logic and integration scripts.
- `flo_humanoid`: Nodes for motor control and simulation of the humanoid robot.
- `flo_vision`: Hand and arm tracking using MediaPipe and OpenCV.
- `flo_core_defs`: Shared message and service definitions.
- `flov2_robot_description`: Robot URDF and description files.
- `flo_face`:

---

## Project Structure

```plaintext
FloSystemV2_GameDemo/
├── docker-compose.yml
├── Dockerfile
├── flo_core/
├── flo_core_defs/
├── flo_face/
├── flo_humanoid/
├── flo_vision/
├── flov2_robot_description/
├── utility_scripts/
└── ...
```

---

## Prerequisites

### Host machine (Ubuntu 22.04)

| Tool                                     | Install command                            |
| ---------------------------------------- | ------------------------------------------ |
| **Docker Engine ≥ 25.0**          | `sudo apt install docker.io`             |
| **docker-compose plugin**          | `sudo apt install docker-compose-plugin` |
| **X-authority helper** (`xhost`) | `sudo apt install x11-xserver-utils`     |

After installing the packages above, add your user to the **docker** group so the container can access X11 and devices without `sudo`:

```bash
sudo usermod -aG docker $USER
# Log out/in or run:
newgrp docker
```

<!-- ## Docker Setup

### 1. Install Docker

Follow the [official Docker installation guide](https://docs.docker.com/get-docker/) for your OS.

### 2. (Optional) Install docker-compose

```sh
sudo apt-get install docker-compose
``` -->

---

## Building and Running with Docker

### Dev computer (mini PC):

1. Setup Ubuntu 22 or 24
2. Install git: `sudo apt install git-all`
3. Clone this repo into `~/Documents/git`
4. Run `./dev_install.sh`

```sh
docker compose build       # ~10 min the first time
docker compose up          # starts a bash prompt inside the catkin_ws
```

---

## Usage

### Running Key Nodes

- **Read and Write Joint Motor Positions:**
  ```sh
  rosrun flo_humanoid read_write_joint_node
  ```
- **Read and Write Single Arm Motor Data:**
  ```sh
  rosrun flo_humanoid read_write_arm_node
  ```
- **Control Both Arms Simultaneously:**
  ```sh
  rosrun flo_humanoid read_write_arms_node
  ```
- **Publish Motor Positions for Testing:**
  ```sh
  rosrun flo_humanoid publish_motor_positions
  ```
- **Subscribe to Motor Position Data:**
  ```sh
  rosrun flo_humanoid subscribe_motor_positions
  ```

### Vision Node Example

- **USB Camera:**
  ```sh
  rosrun usb_cam usb_cam_node _video_device:=/dev/video0
  rosrun flo_vision arm_hand_tracker_node.py _image:=/usb_cam/image_raw _preview:=true _pose:=wave
  ```

---

## License

This project is licensed under the
