# FLO V2 Stack Diagrams

These diagrams describe the current stack in this repository and the main
interaction loop for the Simon Says demo.

## Stack Architecture

```mermaid
flowchart TB
  subgraph Host["Host Machine"]
    operator["Robot Operator\nUses desktop GUI / launches Docker"]
    participant["Participant\nMoves in front of robot"]
    devices["Physical Devices\nUSB camera, motor controller, face controller, audio"]
    x11["X11 / PulseAudio / Device passthrough"]
  end

  subgraph Docker["Docker Container: flo_robot / flo_game"]
    startup["docker_compose_up.sh\n+ ros_docker_auto_startup_launcher.sh"]

    subgraph ROS["ROS Noetic Runtime"]
      roscore["roscore"]

      subgraph Core["flo_core"]
        gui["simon_says_gui.py\nOperator GUI"]
        game["game_runner.py\nSimon Says state machine"]
        moveit["moveit_controller.py\n/simon_cmd action server"]
        convo["conversation_agent.py\nOptional voice conversation node"]
        seq["ActionSequenceController\nPrompt + motion sequencing"]
      end

      subgraph Vision["flo_vision"]
        cam["usb_cam_node\n/usb_cam/image_raw"]
        tracker["arm_hand_tracker_node.py\nPose tracking + calibration"]
      end

      subgraph Humanoid["flo_humanoid"]
        bridge["joint_state_to_dual_arm_motors.py"]
        motors["Dynamixel arm interface"]
      end

      subgraph Face["flo_face"]
        face["face_com_manager.py"]
      end

      subgraph Shared["Shared Definitions / Models"]
        defs["flo_core_defs\nMsgs / srvs / action defs"]
        robot_desc["flov2_robot_description\nURDF / SRDF / meshes"]
        move_group["MoveIt / robot model / planners"]
      end
    end
  end

  operator --> startup
  startup --> roscore
  startup --> gui
  startup --> cam
  startup --> tracker
  startup --> bridge
  startup --> motors
  startup --> face
  startup --> game
  startup --> moveit

  devices --> x11
  x11 --> Docker

  participant --> cam
  cam --> tracker
  tracker --> gui
  tracker --> game

  gui --> game
  game --> moveit
  moveit --> seq
  seq --> move_group
  move_group --> bridge
  bridge --> motors

  game --> face
  convo --> face

  defs --- gui
  defs --- game
  defs --- moveit
  defs --- tracker
  robot_desc --> move_group
```

## ROS Interaction View

```mermaid
flowchart LR
  gui["simon_says_gui.py"]
  game["game_runner.py"]
  tracker["arm_hand_tracker_node.py"]
  moveit["moveit_controller.py"]
  face["face_com_manager.py"]
  convo["conversation_agent.py\noptional"]
  arms["Arm hardware"]
  camera["USB camera"]
  user["Participant"]

  gui -- "/simon_game/control" --> game
  game -- "/simon_game/prompt\n/score\n/turn_id\n/status\n/ui_state" --> gui

  camera --> tracker
  tracker -- "/arm_hand_tracker/pose_score" --> game
  tracker -- "/arm_hand_tracker/preview_image" --> gui

  game -- "/simon_game/calib_cmd" --> tracker
  tracker -- "/simon_game/calib_status" --> game

  game -- "/simon_cmd action goal" --> moveit
  moveit --> arms

  game -- "/emotion" --> face
  convo -- "/conversation/state\n/conversation/face" --> face
  convo -- "/conversation/user_text\n/conversation/assistant_text" --> gui

  user --> camera
  user --> convo
```

## Human-Robot Interaction Flow

```mermaid
flowchart TD
  op["Robot Operator"] -->|"Launches Docker stack\nand opens GUI"| boot["System boot"]
  boot --> robot_ready["Robot stack ready"]
  robot_ready -->|"Clicks setup / calibrate / start"| gui["Simon Says GUI"]

  gui -->|"Control commands"| runner["game_runner"]
  runner -->|"Calibration request"| vision["Vision tracker"]
  participant["Participant"] -->|"Stands in view,\nraises arm,\nrepositions"| vision
  vision -->|"Calibration status + pose scores"| runner
  runner -->|"Prompt text to GUI\nand spoken instructions"| robot["Robot"]
  robot -->|"Arm motion via MoveIt\n+ face expression"| participant

  participant -->|"Imitates action or stays still"| vision
  vision -->|"Pose match / similarity"| runner
  runner -->|"Update score, turn, status"| gui
  gui -->|"Displays progress to operator"| op

  op -->|"Pause / resume / replay / stop if needed"| gui
  runner -->|"Next round or game over"| robot
  robot -->|"Final feedback"| participant
```

## Role Summary

- `Robot operator`: starts the stack, monitors the GUI, and controls game flow.
- `Participant`: responds to Flo's instructions and is tracked by the vision node.
- `Robot`: combines arm motion, face output, and spoken prompts to run the activity.
- `Vision system`: closes the loop by checking whether the participant matched the expected pose.

