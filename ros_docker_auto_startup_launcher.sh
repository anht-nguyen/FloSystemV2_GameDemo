
#!/usr/bin/env bash
set -e

# Auto startup script for ROS on ubuntu 20 machine (non-docker) in a tmux session

# 0) load ROS & workspace overlays
# source /opt/ros/noetic/setup.bash
# source /catkin_ws/devel/setup.bashem
# udevadm control --reload-rules && udevadm trigger # ensure udev inside the container sees the camera

# 1) Start tmux session named “ros”
tmux new-session -d -s ros #'roscore'

# wait for master
sleep 5

# 2) In new tmux windows, launch your files
tmux new-window -t ros:1 -n robot_sim 'roslaunch flo_core full_robot_arm_sim.launch show_gz_gui:=true use_rviz:=true' #launch the robot simulation
sleep 2
tmux new-window -t ros:2 -n camera 'roslaunch flo_vision usb_cam_launcher.launch' #'roslaunch astra_camera astra.launch' # uncomment and complete with camera launch file
sleep 2
tmux new-window -t ros:3 -n vision 'roslaunch flo_vision arm_hand_tracker_launcher.launch' #uncomment to run the arm hand tracker node
sleep 2
tmux new-window -t ros:4 -n vision_monitor 'rostopic echo /arm_hand_tracker/pose_score' #uncomment to monitor the arm hand tracker node
sleep 5
tmux new-window -t ros:5 -n hw_if       'roslaunch flo_humanoid dual_arm_hardware.launch'
sleep 2
tmux new-window -t ros:6 -n face 'roslaunch flo_face flo_face_launcher.launch' 
sleep 2
tmux new-window -t ros:7 -n game_runner 'roslaunch flo_core simonsays_launcher_prod.launch'



# 3) (optional) Attach so you see the tmux panes on “docker attach”
# tmux attach -t ros


# (optional) leave tmux detached, then block forever
# exec tail -f /dev/null
