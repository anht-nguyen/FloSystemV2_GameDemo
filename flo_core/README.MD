# flo_core

ROS package for the Flo V2 Simon Says game, MoveIt-based dual-arm motion
control, and a desktop GUI.

## Key functionality

- `scripts/game_runner.py`: SMACH-based Simon Says executive. Handles intro
  flow, calibration handshake, turn sequencing, scoring, and pause/resume.
  Publishes `/simon_game/prompt`, `/simon_game/score`, `/simon_game/turn_id`,
  and `/emotion`. Subscribes to `/simon_game/control`,
  `/simon_game/calib_status`, and `/arm_hand_tracker/pose_score`. Sends
  `/simon_cmd` action goals.
- `scripts/moveit_controller.py`: Provides the `/simon_cmd` action server.
  Parses gesture strings like `<ACTION>_left|<ACTION>_right` and executes
  dual-arm motions via `ActionSequenceController`. Also listens for legacy
  `/action_command` messages.
- `scripts/simon_says_gui.py`: Qt GUI that publishes `/simon_game/control`
  and displays prompt, score, turn, timer, and status.

## Core modules

- `src/flo_core/action_sequence_controller.py`: Action enum and MoveIt
  action sequences, including dual-arm execution.
- `src/flo_core/prompt_utils.py`: Builds natural language prompts for
  left/right actions with optional "Simon says" prefix.
- `src/flo_core/polly_tts_streaming.py`: Streaming text-to-speech via
  Amazon Polly and PyAudio.
- `src/flo_core/polly_tts.py`: File-based Polly TTS helper.

## Configuration

- `config/simonsays_game_params.yaml`: Allowed actions, timeouts, thresholds,
  and round count for the game.

## Launch

- `launch/simonsays_launcher_prod.launch`: Loads game parameters and starts
  `moveit_controller`, `game_runner`, and the GUI.
- Other launch files under `launch/` support MoveIt, Gazebo, and RViz setups
  used by the broader Flo V2 stack.

## Package files

- `CMakeLists.txt`, `package.xml`, `setup.py`: Build and package metadata.

## Quick start

```bash
roslaunch flo_core simonsays_launcher_prod.launch
```

## Dependencies

- ROS (e.g., Noetic), MoveIt, and `flo_core_defs` messages/actions.
- `PyQt5` for the GUI.
- `boto3` and `pyaudio` for Polly TTS.
- A pose-scoring publisher (e.g., `flo_vision`) on
  `/arm_hand_tracker/pose_score` and calibration status on
  `/simon_game/calib_status`.
