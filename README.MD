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

### Setting up Host machine

1. Install Ubuntu 22 or 24
2. Install [docker](https://docs.docker.com/engine/install/ubuntu/) and complete the post installation steps, then test docker by running the hello world image
   Note: after installing Docker engine and Docker Compose plugin do the following:

   a. Enable and start the Docker service

   ```sh
   sudo systemctl enable docker
   sudo systemctl start docker
   ```

   b. Add your user to the docker group (to run without sudo)

   ```sh
   sudo usermod -aG docker $USER
   ```

   c. Verify installation: `docker run hello-world`, `docker compose version`
3. Install git: `sudo apt install git-all`
4. Clone this repo into `~/Documents/git`
5. Build Docker containner: `docker compose up`
6. Automatically startup docker and the game:

   1. Make startup launcher script executable: `chmod +x ./auto_startup_flogame_launcher.sh`



Set up AWS Polly: You need to setup [Amazon Polly](#amazon-polly) This should be as easy as
just answering the questions during the install script.

---

## Building and Running with Docker

```sh
docker compose build       # ~10 min the first time
docker compose up          # starts a bash prompt inside the catkin_ws
```

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

## Misc.

### Amazon Polly

Polly will need to have access to AWS to work. To set that up, go to the
[IAM Console](https://console.aws.amazon.com/iam) and click on users. Then
select the user you want to give access to. Click "Create access key" under
Security Credentials. Then in the console on the computer run `./aws_install.sh`
which will
[install the AWS CLI](https://docs.aws.amazon.com/cli/latest/userguide/cli-chap-install.html)
for you and run `aws configure --profile flo`, it will prompt you for input an
you should fill in the info from the IAM console with region set to us-east-1
and output set to json.

A few notes:

- you should probably restrict what the user that is getting access can do. The
  best option is to set the permissions to AmazonPollyReadOnlyAccess
- You can have other users configured. By changing the --profile, you control
  the name of the user if no profile is specified, then the information is saved
  to a default
- If you ever need to revoke permissions for a user, that can be done from the
  IAM Console by deactivating the user's Access Keys.
- TODO: I would like to find a way to limit how many calls can be made by a
  user.
- To check whether polly is available given the profile, run
  `aws polly help --profile flo`. If you get back a list of commands to run on
  polly, you should be good.
- To fully test, can run:
  `aws polly synthesize-speech --output-format mp3 --voice-id Ivy --text 'hello, this is a test' --profile flo test.mp3`
  and then play the audio by installing: `sudo apt install mpg123` and typing
  `mpg123 test.mp3`
- you might find that you are getting some sort of server connection errors. you
  can resolve that by running `pip3 install -U boto3` (this should now be a part
  of the install script, but it is commented out)

---

## License

TBD
