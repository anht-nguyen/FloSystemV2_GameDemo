# flo_vision – Hand & Arm Tracker for FLO Robot

This package provides a MediaPipe-based hand and arm tracker for the FLO robot, integrated with ROS Noetic. It enables real-time tracking of arm and hand joint angles using a standard USB webcam.

## Features

- Real-time hand and arm joint tracking using MediaPipe and OpenCV.
- Publishes joint angles as ROS topics for integration with other robot components.
- Easy-to-use ROS node and launch files.

## Prerequisites

- **ROS Noetic** installed and sourced.
- **USB webcam** connected to your computer or robot.
- The following ROS and system dependencies:
  - `usb_cam` ROS package
  - `cv_bridge`, `image_transport`, `sensor_msgs`, `std_msgs`
  - Python libraries: `opencv-python`, `mediapipe`

## Installation

1. **Install ROS dependencies:**

   ```sh
   sudo apt-get install ros-noetic-usb-cam ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-sensor-msgs ros-noetic-std-msgs
   source /opt/ros/noetic/setup.bash
   ```
2. **Install Python dependencies:**

   ```sh
   pip3 install opencv-python mediapipe
   ```
3. **Build the package:**

   Make sure you are in your catkin workspace root (e.g., `~/catkin_ws_flogamedemo`):

   ```sh
   cd ~/catkin_ws_flogamedemo
   catkin build flo_vision
   source devel/setup.bash
   ```

## Usage

### **Option A: Using rosrun on USB Camera**

1. Start ROS Core

```sh
roscore
```

2. Start the USB Camera Node

```sh
rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _image_width:=1280  _image_height:=720 _framerate:=30 _pixel_format:=yuyv image_raw:=/usb_cam/image_raw 

```

3. Run the Arm & Hand Tracker Node (Change the pose by editing _pose:= `<pose_to_detect>`

```sh
rosrun flo_vision arm_hand_tracker_node.py _image:=/usb_cam/image_raw _preview:=true _pose:=wave
```

- `_image` argument specifies the input image topic (default: `/usb_cam/image_raw`).
- `_preview` argument enables a GUI preview window (set to `false` to disable).

4. If want to change the pose to detect during the node running, publish the pose to the topic below (e.g. "raise", "wave" or "raise, wave"):

```
rostopic pub /arm_hand_tracker/pose_command std_msgs/String "data: 'raise, wave'" -1
```

### **Option B: Using rosrun on Astra S Camera**

If first time using Astra S on this PC:

* Astra S Camera parameter [here](https://store.orbbec.com/products/astra-s?srsltid=AfmBOoo2ISCxdDyZAGINGOiX-Wj8_nkPNW2mVdd6Mx7xYWaItNm0fXpg)
* Install the Astra Ubuntu SDK [here](https://www.orbbec.com/developers/astra-sdk/) and use the version of **Ubuntu 18.04**
* Clone the ROS Astra Driver Package and rebuild your catkin workspace

  ```
  cd ~/catkin_ws/scr
  git clone https://github.com/orbbec/ros_astra_camera
  cd ~/catkin_ws_flogamedemo
  catkin build astra_camera
  source devel/setup.bash
  ```

1. Start ROS Core

```sh
roscore
```

2. Start the Astra Camera

```sh
roslaunch astra_camera astra.launch
```

3. Run the Arm & Hand Tracker Node (Change the pose by editing _pose:= `<pose_to_detect>`

```sh
rosrun flo_vision arm_hand_tracker_node.py _image:=/camera/color/image_raw _preview:=true _pose:=wave
```


### **Option C: Using launch file on USB Camera**

```sh
roslaunch flo_vision arm_hand_tracker_with_webcam.launch
```

This launch file will automatically start the tracker node with recommended parameters.

## Troubleshooting

- Check Published Topics: To verify that the node is publishing joint angles, use:

```sh
rostopic echo -n1 /arm_hand_tracker/left_elbow_angle
```

You should see numeric output if the tracker is running and detecting joints.

- Ensure your webcam is connected and accessible as `/dev/video0`.
- If you see no output on the joint angle topics, check that the camera node and tracker node are both running and that the image topic names match.
- For additional debug information, run the tracker node with `_preview:=true` to see the processed video stream.
- Testing `usb_cam`:
  To verify that the `usb_cam` node is working correctly, you can launch the provided test file:

  ```sh
  roslaunch usb_cam usb_cam-test.launch
  ```

  This should start the camera node and display the video stream, confirming that your webcam is accessible.

## License

MIT License

---

For more details, see the source code and launch files in the [flo_vision](flo_vision) package.
