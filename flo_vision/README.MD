sudo apt-get install ros-noetic-usb-cam

roslaunch usb_cam usb_cam-test.launch


rostopic echo -n1 /arm_hand_tracker/left_elbow_angle   # see if any numbers ever appear



Steps: 

roscore

rosrun usb_cam usb_cam_node         _video_device:=/dev/video0         _width:=640 _height:=480         _framerate:=30         _pixel_format:=yuyv         image_raw:=/usb_cam/image_raw

rosrun flo_vision arm_hand_tracker_node.py _image:=/usb_cam/image_raw _preview:=true