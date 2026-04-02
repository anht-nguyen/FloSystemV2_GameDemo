FROM ros:noetic-ros-core

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS_DISTRO=noetic
ARG UID=1000
ARG GID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    CATKIN_WS=/catkin_ws \
    AWS_DEFAULT_REGION=us-east-1

WORKDIR ${CATKIN_WS}

# System, ROS, audio, and build dependencies are installed together to keep the
# image setup predictable and reduce repeated apt metadata downloads.
RUN set -eux; \
    apt-get update; \
    apt-get upgrade -y; \
    apt-get install -y \
        alsa-utils \
        build-essential \
        git \
        htop \
        libasound2 \
        libasound2-plugins \
        libpulse0 \
        nano \
        portaudio19-dev \
        pulseaudio-utils \
        python3-catkin-tools \
        python3-matplotlib \
        python3-numpy \
        python3-pip \
        python3-protobuf \
        python3-pyqt5 \
        python3-rosdep \
        python3-tk \
        ros-${ROS_DISTRO}-controller-manager \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-dynamixel-sdk \
        ros-${ROS_DISTRO}-dynamixel-sdk-examples \
        ros-${ROS_DISTRO}-gazebo-ros-control \
        ros-${ROS_DISTRO}-gazebo-ros-pkgs \
        ros-${ROS_DISTRO}-image-geometry \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-joint-state-controller \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-moveit \
        ros-${ROS_DISTRO}-rgbd-launch \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-ros-control \
        ros-${ROS_DISTRO}-ros-controllers \
        ros-${ROS_DISTRO}-rosbag \
        ros-${ROS_DISTRO}-rosbridge-server \
        ros-${ROS_DISTRO}-rosserial-arduino \
        ros-${ROS_DISTRO}-rosserial-client \
        ros-${ROS_DISTRO}-rosserial-msgs \
        ros-${ROS_DISTRO}-rosserial-python \
        ros-${ROS_DISTRO}-rosserial-server \
        ros-${ROS_DISTRO}-sensor-msgs \
        ros-${ROS_DISTRO}-smach \
        ros-${ROS_DISTRO}-smach-ros \
        ros-${ROS_DISTRO}-smach-viewer \
        ros-${ROS_DISTRO}-sound-play \
        ros-${ROS_DISTRO}-std-msgs \
        ros-${ROS_DISTRO}-usb-cam \
        tmux \
        udev \
        x11-apps; \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --default-timeout=100 \
        absl-py \
        awscli \
        boto3 \
        matplotlib \
        numpy \
        opencv-contrib-python \
        opencv-python \
        pyaudio \
        scipy && \
    python3 -m pip install --no-cache-dir --no-deps mediapipe==0.10.11

RUN mkdir -p "${CATKIN_WS}/src" /root/.aws

COPY certs/aws-credentials /root/.aws/credentials
COPY certs/aws-config /root/.aws/config

RUN chmod 600 /root/.aws/credentials /root/.aws/config

RUN printf '%s\n' \
    "default-server = unix:/tmp/pulse/native" \
    "autospawn = no" \
    "daemon-binary = /bin/true" \
    "enable-shm = false" \
    > /etc/pulse/client.conf && \
    printf '%s\n' \
    'pcm.!default {' \
    '  type pulse' \
    '  fallback "sysdefault"' \
    '}' \
    'ctl.!default {' \
    '  type pulse' \
    '}' \
    > /etc/asound.conf

COPY . ${CATKIN_WS}/src/
COPY ros_docker_auto_startup_launcher.sh /usr/local/bin/ros_docker_auto_startup_launcher.sh

RUN chmod +x /usr/local/bin/ros_docker_auto_startup_launcher.sh && \
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin build -DCMAKE_BUILD_TYPE=Release

RUN set -eux; \
    groupadd --gid "${GID}" hostgroup; \
    useradd --uid "${UID}" --gid hostgroup --create-home --shell /bin/bash hostuser; \
    mkdir -p /home/hostuser/.aws; \
    cp /root/.aws/credentials /home/hostuser/.aws/credentials; \
    cp /root/.aws/config /home/hostuser/.aws/config; \
    chown -R hostuser:hostgroup "${CATKIN_WS}" /home/hostuser/.aws; \
    chown -R hostuser:hostgroup /usr/local/lib/python3.8/dist-packages/mediapipe

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    echo "source ${CATKIN_WS}/devel/setup.bash" >> /etc/bash.bashrc

# ENTRYPOINT ["/usr/local/bin/ros_docker_auto_startup_launcher.sh"]
