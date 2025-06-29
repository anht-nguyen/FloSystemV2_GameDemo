# ------------  base OS & ROS --------------
# Ubuntu 20.04 + GUI tools + MoveIt
# official tags: hub.docker.com/_/ros

FROM ros:noetic-ros-core
ENV DEBIAN_FRONTEND=noninteractive

# Install desktop and MoveIt dependencies explicitly to avoid unnecessary packages
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    ros-noetic-desktop \
    ros-noetic-moveit \
    ros-noetic-sound-play \
    # new: hardware & controller support
    ros-noetic-dynamixel-sdk ros-noetic-dynamixel-sdk-examples \
    ros-noetic-controller-manager ros-noetic-joint-state-controller \
    ros-noetic-joint-state-publisher ros-noetic-robot-state-publisher \
    # new: vision & message transport
    ros-noetic-usb-cam ros-noetic-cv-bridge ros-noetic-image-transport \
    ros-noetic-sensor-msgs ros-noetic-std-msgs \
    # misc tooling
    python3-pip python3-pyqt5 alsa-utils \
    git \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# ------------  Python user-level deps -----
RUN pip3 install --no-cache-dir \
    boto3 \
    pyaudio \
    opencv-python \
    mediapipe

# ------------  Catkin workspace ----------
ENV CATKIN_WS=/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS

# clone repo into workspace
RUN apt-get update && apt-get install -y --no-install-recommends git && \
    git clone --branch master https://github.com/anht-nguyen/FloSystemV2_GameDemo.git $CATKIN_WS/src/FloSystemV2_GameDemo && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# resolve rosdep and build
RUN . /opt/ros/noetic/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make -DCMAKE_BUILD_TYPE=Release

# ------------  source overlays -----------
RUN echo "source /opt/ros/noetic/setup.bash"   >> /etc/bash.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /etc/bash.bashrc

CMD ["bash"]
