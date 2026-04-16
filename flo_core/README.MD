# flo_core

ROS package for the Flo V2 Simon Says game, MoveIt-based dual-arm motion
control, and a desktop GUI.

## Key functionality

- `scripts/conversation_agent.py`: Real-time conversational node that asks a
  question, records user speech from the microphone, sends it to the
  Gemini Developer API for transcription, generates a spoken follow-up with
  a chat model, and plays the reply through Amazon Polly.
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
- `src/flo_core/conversation_agent.py`: Conversation loop, microphone capture,
  STT/chat API client, and ROS topic bridge for text/state events.
- `src/flo_core/polly_tts_streaming.py`: Streaming text-to-speech via
  Amazon Polly and PyAudio.
- `src/flo_core/polly_tts.py`: File-based Polly TTS helper.

## Configuration

- `config/simonsays_game_params.yaml`: Allowed actions, timeouts, thresholds,
  and round count for the game.

## Launch

- `launch/simonsays_launcher_prod.launch`: Loads game parameters and starts
  `moveit_controller`, `game_runner`, and the GUI.
- `launch/conversation_agent.launch`: Starts the conversational interaction
  node using Polly for TTS and the Gemini API for transcription and
  dialogue generation.
- Other launch files under `launch/` support MoveIt, Gazebo, and RViz setups
  used by the broader Flo V2 stack.

## Package files

- `CMakeLists.txt`, `package.xml`, `setup.py`: Build and package metadata.

## Quick start

```bash
roslaunch flo_core simonsays_launcher_prod.launch
```

## Conversation demo

Set a Gemini API key for the language model service:

```bash
export GEMINI_API_KEY=your_key_here
```

Or store it in:

```bash
certs/gemini-api-key
```

Then launch the conversational node:

```bash
roslaunch flo_core conversation_agent.launch
```

Useful params:

- `~opening_prompt`: First thing Flo says before listening.
- `~listen_duration_s`: Fixed microphone capture length for each turn.
- `~record_device`: Optional ALSA device name passed to `arecord`.
  Defaults to `FLO_RECORD_DEVICE`, then `AUDIODEV`, then `pulse`.
- `~silence_rms_threshold`: Minimum audio RMS before Flo sends the clip for transcription.
  Increase it if room noise is causing false triggers; set to `0` to disable silence detection.
- `~chat_model`: Chat-completion model name.
- `~transcription_model`: Audio transcription model name.
  Defaults to `gemini-2.5-flash` unless overridden by environment or launch params.
- `~api_key_file`: Path to a file containing only the API key.
- `~loop`: Whether Flo keeps taking turns until a stop phrase is spoken.

Published topics:

- `/conversation/user_text`
- `/conversation/assistant_text`
- `/conversation/state`

Optional manual input topic:

- Publish `std_msgs/String` to `/conversation/input_text` to inject typed text
  without using the microphone.

## Dependencies

- ROS (e.g., Noetic), MoveIt, and `flo_core_defs` messages/actions.
- `PyQt5` for the GUI.
- `boto3` and `pyaudio` for Polly TTS.
- Outbound HTTPS access to the Gemini Developer API; the conversation agent uses the REST API directly, so no extra Gemini Python SDK is required.
- `arecord` from `alsa-utils` for microphone capture.
- A pose-scoring publisher (e.g., `flo_vision`) on
  `/arm_hand_tracker/pose_score` and calibration status on
  `/simon_game/calib_status`.
