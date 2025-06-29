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
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-rosbag \
    ros-noetic-smach \
    ros-noetic-smach-ros \  
    ros-noetic-smach-viewer \
    ros-noetic-rosbridge-server \
    # new: vision & message transport
    ros-noetic-usb-cam ros-noetic-cv-bridge ros-noetic-image-transport \
    ros-noetic-sensor-msgs ros-noetic-std-msgs \
    # misc tooling
    python3-pip python3-pyqt5 alsa-utils \
    portaudio19-dev \
    build-essential \
    git \
    python3-rosdep \
    python3-catkin-tools \
    python3-matplotlib \
    python3-numpy \
 && apt-get clean && rm -rf /var/lib/apt/lists/*


 RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    python-is-python3 \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# ------------  Python user-level deps -----
RUN pip3 install --default-timeout=100 \
    numpy \
    scipy \
    matplotlib \
    boto3 \
    pyaudio \
    opencv-python 

RUN pip3 install --default-timeout=100 mediapipe

# ------------  Catkin workspace ----------
ENV CATKIN_WS=/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS

# Copy your code (instead of git-cloning inside the container)
#    Context path must include this Dockerfile and your repo
COPY . /$CATKIN_WS/src/

# resolve rosdep and build
RUN rosdep init 

RUN . /opt/ros/noetic/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make -DCMAKE_BUILD_TYPE=Release

# ------------  source overlays -----------
RUN echo "source /opt/ros/noetic/setup.bash"   >> /etc/bash.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /etc/bash.bashrc

CMD ["bash"]
