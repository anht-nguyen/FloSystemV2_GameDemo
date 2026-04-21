# FloSystemV2_GameDemo

ROS Noetic workspace for the FLO V2 robot demo, including dual-arm control, face control, camera-based pose tracking, MoveIt/Gazebo simulation, and the Simon Says game flow.

## What This Repo Includes

- `flo_core`: Simon Says game runner, MoveIt action server, GUI, Gazebo and MoveIt launch files.
- `flo_humanoid`: Dynamixel-based arm hardware interface and motor-control nodes.
- `flo_vision`: USB camera launcher and MediaPipe-based arm/hand pose tracker.
- `flo_face`: Face display launcher and serial communication support.
- `flo_core_defs`: Shared ROS messages, services, and actions.
- `flov2_robot_description`: URDF, SRDF, meshes, and robot description assets.
- `utility/`: Host-side helpers for device-path resolution and udev rule generation.

## Current Startup Flow

The current Docker workflow launches the full demo stack automatically inside a `tmux` session:

1. `roscore`
2. robot simulation via `roslaunch flo_core full_robot_arm_sim.launch`
3. camera via `roslaunch flo_vision usb_cam_launcher.launch`
4. pose tracking via `roslaunch flo_vision arm_hand_tracker_launcher.launch`
5. pose-score monitor
6. arm hardware interface via `roslaunch flo_humanoid dual_arm_hardware.launch`
7. face launcher via `roslaunch flo_face flo_face_launcher.launch`
8. Simon Says game via `roslaunch flo_core simonsays_launcher_prod.launch`

That behavior is defined in [`ros_docker_auto_startup_launcher.sh`](/home/rrl/FloSystemV2_GameDemo/ros_docker_auto_startup_launcher.sh).

## Requirements

- Ubuntu host machine
- Docker Engine with the Docker Compose plugin
- Git
- X11 access for GUI apps
- PulseAudio or PipeWire audio on the host
- Optional but recommended:
  - USB camera
  - motor controller connected over serial
  - face controller connected over serial
  - AWS credentials for Amazon Polly

## Quick Start With Docker

### 1. Clone the repo

```bash
git clone https://github.com/anht-nguyen/FloSystemV2_GameDemo.git
cd FloSystemV2_GameDemo
```

### 2. Install Docker prerequisites

```bash
sudo apt update
sudo apt install -y docker.io docker-compose-plugin git
sudo systemctl enable docker
sudo systemctl start docker
sudo usermod -aG docker "$USER"
newgrp docker
```

### 3. Create AWS credential files

The Docker image expects local files under `certs/`, and `docker-compose.yml` bind-mounts them into the container.

```bash
mkdir -p certs
```

Create `certs/aws-credentials`:

```ini
[flo]
aws_access_key_id = YOUR_AWS_KEY_ID
aws_secret_access_key = YOUR_AWS_SECRET_KEY
```

Create `certs/aws-config`:

```ini
[profile flo]
region = us-east-1
output = json
```

If you are not using Polly yet, create placeholder files so the Docker build and bind mounts still succeed.

### 4. Resolve device paths

The repo now includes a device helper that exports stable environment variables used by Docker and ROS launch files:

```bash
eval "$(python3 utility/device_paths.py --format shell)"
```

This sets:

- `FLO_CAMERA_DEVICE`
- `FLO_MOTORS_DEVICE`
- `FLO_FACE_DEVICE`
- `FLO_CAMERA_SOURCE`
- `FLO_MOTORS_SOURCE`
- `FLO_FACE_SOURCE`

By default it uses:

- udev symlinks if `/etc/udev/rules.d/99-flo-devices.rules` exists
- otherwise legacy paths such as `/dev/video0`, `/dev/ttyUSB0`, and `/dev/ttyACM0`

### 5. Build and start

Before starting GUI apps from Docker, allow local X11 access:

```bash
xhost +local:
./docker_compose_up.sh
```

### 6. Inspect the running stack

The container stays up and launches ROS processes in `tmux`.

```bash
docker exec -it flo_game tmux ls
docker exec -it flo_game tmux attach -t ros
```

To stop everything:

```bash
docker compose down
```

To recreate the container with refreshed device-path resolution:

```bash
docker compose down
./docker_compose_up.sh
```

## Device Setup

### Recommended: persistent udev rules

To avoid device-number changes across reboots, generate stable symlinks on the host:

```bash
sudo python3 utility/create_udev_rules.py \
  --camera /dev/video0 \
  --motors /dev/ttyUSB0 \
  --face /dev/ttyACM0 \
  -o /etc/udev/rules.d/99-flo-devices.rules

sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/flo_camera /dev/flo_motors /dev/flo_face
```

Once installed, `utility/device_paths.py` will prefer `/dev/flo_camera`, `/dev/flo_motors`, and `/dev/flo_face`.

If your webcam exposes more than one `/dev/video*` node, pass the actual capture
device to `--camera` when generating the rule. The helper now records capture-specific
matchers so `/dev/flo_camera` stays attached to the stream node instead of drifting
to a metadata sibling.

## Simon Says Demo

The main game launch file is:

```bash
roslaunch flo_core simonsays_launcher_prod.launch
```

That launch file starts:

- `moveit_controller.py`
- `game_runner.py`
- `simon_says_gui.py`

Game parameters are loaded from [`flo_core/config/simonsays_game_params.yaml`](/home/rrl/FloSystemV2_GameDemo/flo_core/config/simonsays_game_params.yaml).

## Conversation Demo

The repo now also includes a conversational ROS node that can:

- speak a prompt using Amazon Polly
- record microphone audio with `arecord`
- transcribe speech with the Gemini API
- generate a follow-up conversational reply with Gemini
- speak the response back through the existing Polly audio path

Run it with:

```bash
export GEMINI_API_KEY=your_key_here
export GEMINI_CHAT_MODEL=${GEMINI_CHAT_MODEL:-gemini-2.5-flash}
export GEMINI_TRANSCRIPTION_MODEL=${GEMINI_TRANSCRIPTION_MODEL:-gemini-2.5-flash}
roslaunch flo_core conversation_agent.launch
```

You can also store the key in `certs/gemini-api-key`.

Useful runtime params include `chat_model`, `transcription_model`,
`listen_duration_s`, `record_device`, and `opening_prompt`.

For Docker microphone access, the stack defaults `record_device` to `pulse`.
If you need to target a raw ALSA capture device instead, export something like:

```bash
export FLO_RECORD_DEVICE=hw:1,0
```

Quick microphone smoke test inside the running container:

```bash
docker exec -it flo_game arecord -D "${FLO_RECORD_DEVICE:-pulse}" -f S16_LE -r 16000 -c 1 -d 3 /tmp/mic_test.wav
```

## Common Commands

### Run the vision stack manually

```bash
eval "$(python3 utility/device_paths.py --format shell)"
roslaunch flo_vision usb_cam_launcher.launch
roslaunch flo_vision arm_hand_tracker_launcher.launch
```

### Run the dual-arm hardware interface

```bash
roslaunch flo_humanoid dual_arm_hardware.launch
```

### Run the face controller

```bash
roslaunch flo_face flo_face_launcher.launch
```

### Exercise demo arm motions

```bash
./demo_show_actions.sh --side right --repeat 1
```

## Host Auto-Startup

The repo now includes a host-side startup path for kiosk/demo deployments:

- [`host_auto_startup_launcher.sh`](/home/rrl/FloSystemV2_GameDemo/host_auto_startup_launcher.sh): waits for network, updates the repo, resolves devices, then starts Docker Compose
- [`flo_game.service`](/home/rrl/FloSystemV2_GameDemo/flo_game.service): systemd unit for launching the demo at boot

Important: [`flo_game.service`](/home/rrl/FloSystemV2_GameDemo/flo_game.service) is currently checked in with `User=rrl` and paths under `/home/rrl/...`. Update those values before installing it on another machine.

Install example:

```bash
sudo cp host_auto_startup_launcher.sh /usr/local/bin/host_auto_startup_launcher.sh
sudo chmod 755 /usr/local/bin/host_auto_startup_launcher.sh
sudo cp flo_game.service /etc/systemd/system/flo_game.service
sudo systemctl daemon-reload
sudo systemctl enable flo_game.service
```

Check logs after reboot:

```bash
systemctl status flo_game.service
tail -n 100 "$HOME/flo_game_startup.log"
```

## Non-Docker Development

For host-native setup, use [`dev_install.sh`](/home/rrl/FloSystemV2_GameDemo/dev_install.sh). It currently targets Ubuntu 22.04-style developer bootstrapping and installs:

- Docker and desktop helpers
- ROS Noetic binaries
- catkin tools and rosdep
- Astra/OpenNI-related dependencies

After setup, the non-Docker tmux launcher is:

```bash
./nondocker_ros_auto_startup_launcher.sh
```

## Repo Layout

```text
FloSystemV2_GameDemo/
├── Dockerfile
├── docker-compose.yml
├── flo_core/
├── flo_core_defs/
├── flo_face/
├── flo_humanoid/
├── flo_vision/
├── flov2_robot_description/
├── utility/
├── ros_docker_auto_startup_launcher.sh
├── nondocker_ros_auto_startup_launcher.sh
├── host_auto_startup_launcher.sh
└── flo_game.service
```

## Notes

- The stack currently targets ROS Noetic.
- The Docker image builds the catkin workspace during image creation.
- `docker-compose.yml` runs the container in privileged mode so camera, serial, audio, and hot-plugged devices can be passed through cleanly.
- GUI, audio, and AWS credentials all rely on host-side environment and mounted files being present.

## License

License information has not been finalized in this repository yet.
