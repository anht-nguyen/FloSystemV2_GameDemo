# ---------- Base OS & ROS ----------
FROM ros:noetic-ros-core
ENV DEBIAN_FRONTEND=noninteractive

# ---------- Core desktop + MoveIt + common runtime ----------
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
        ros-noetic-desktop \
        ros-noetic-moveit \
        ros-noetic-sound-play \
        ros-noetic-dynamixel-sdk ros-noetic-dynamixel-sdk-examples \
        ros-noetic-controller-manager ros-noetic-joint-state-controller \
        ros-noetic-joint-state-publisher ros-noetic-robot-state-publisher \
        ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
        ros-noetic-ros-control ros-noetic-ros-controllers \
        ros-noetic-rosbag \
        ros-noetic-smach ros-noetic-smach-ros ros-noetic-smach-viewer \
        ros-noetic-rosbridge-server \
        # --- Vision ---
        ros-noetic-usb-cam ros-noetic-cv-bridge ros-noetic-image-transport \
        ros-noetic-sensor-msgs ros-noetic-std-msgs \
        # --- Utilities ---
        python3-pip python3-pyqt5 alsa-utils portaudio19-dev \
        build-essential git python3-rosdep python3-catkin-tools \
        python3-matplotlib python3-numpy tmux x11-apps htop python3-protobuf\
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ---------- Python packages ----------
RUN pip3 install --default-timeout=100 \
        numpy scipy matplotlib boto3 pyaudio opencv-python absl-py && \
    pip3 install --no-deps mediapipe==0.10.11 opencv-contrib-python

# ---------- Extra ROS packages ----------
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-noetic-image-geometry \
        ros-noetic-rgbd-launch \
        udev \
        ros-noetic-rosserial-arduino ros-noetic-rosserial-client \
        ros-noetic-rosserial-server ros-noetic-rosserial-python \
        ros-noetic-rosserial-msgs \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ---------- AWS CLI ----------
RUN pip3 install awscli
RUN mkdir -p /root/.aws
COPY certs/aws-credentials /root/.aws/credentials
COPY certs/aws-config       /root/.aws/config
RUN chmod 600 /root/.aws/*
ENV AWS_DEFAULT_REGION=us-east-1    

# ---------- Install dependencies ----------
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-tk \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ---------- Install PulseAudio client libraries ----------
# audio client libraries only – NOT the daemon
RUN apt-get update && apt-get install -y --no-install-recommends \
        libpulse0 alsa-utils libasound2 libasound2-plugins \
        pulseaudio-utils && \
    rm -rf /var/lib/apt/lists/*

# keep Pulse from autospawning inside the container
RUN printf '%s\n' \
    "default-server = unix:/tmp/pulse/native" \
    "autospawn      = no" \
    "daemon-binary  = /bin/true" \
    "enable-shm     = false" \
    > /etc/pulse/client.conf

# ----- Route every ALSA client to Pulse -----------------------------
RUN printf '%s\n' \
    'pcm.!default {' \
    '  type pulse        # talk to the host Pulse / PipeWire server' \
    '  fallback "sysdefault"' \
    '}' \
    'ctl.!default {' \
    '  type pulse' \
    '}' \
    > /etc/asound.conf

# ---------- Catkin workspace ----------
ENV CATKIN_WS=/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS
COPY . $CATKIN_WS/src/

# ---------- Startup script ----------
COPY ros_docker_auto_startup_launcher.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/ros_docker_auto_startup_launcher.sh

# ---------- Build workspace ----------
RUN rosdep init && \
    . /opt/ros/noetic/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin build -DCMAKE_BUILD_TYPE=Release

# ---------- User setup ----------
# Create a user with the same UID/GID as the host user
# This allows the container to access host files with the same permissions
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} hostgroup && \
    useradd  -u ${UID} -g hostgroup -m -s /bin/bash hostuser

RUN chown -R hostuser:hostgroup /catkin_ws

# ---------- give hostuser write access where it is needed ----------
# 1. MediaPipe model directory
RUN chown -R hostuser:hostgroup \
    /usr/local/lib/python3.8/dist-packages/mediapipe

# 2. copy AWS credentials to the unprivileged home
RUN mkdir -p /home/hostuser/.aws && \
    cp /root/.aws/credentials /home/hostuser/.aws/ && \
    cp /root/.aws/config       /home/hostuser/.aws/ && \
    chown -R hostuser:hostgroup /home/hostuser/.aws

# ---------- Convenience sources ----------
RUN echo "source /opt/ros/noetic/setup.bash"   >> /etc/bash.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /etc/bash.bashrc

# ENTRYPOINT ["/usr/local/bin/ros_docker_auto_startup_launcher.sh"]
