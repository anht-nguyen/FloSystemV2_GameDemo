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
    tmux \
    x11-apps \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

####################  Orbbec Astra SDK & driver  ####################
# 1) System-level deps for OpenNI2 + libuvc
# ── prerequisites ───────────────────────────────────────────────
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libusb-1.0-0-dev libudev-dev libglfw3-dev libopencv-dev \
        libopenni2-dev openni2-utils unzip curl git wget bzip2 ca-certificates && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-noetic-backward-ros \
    && rm -rf /var/lib/apt/lists/*

ENV OPENNI2_REDIST=/usr/lib   
# ← add once, use everywhere

# ── Orbbec OpenNI 2.3.0.86 (beta-6) install ─────────────────────
# ──   Download once – cached ────────────────────────────────────────────
RUN curl -L -o /tmp/openni.zip \
        https://dl.orbbec3d.com/dist/openni2/v2.3.0.86-beta6/Orbbec_OpenNI_v2.3.0.86-beta6_linux_release.zip && \
    mkdir -p /tmp/openni && \
    unzip -q /tmp/openni.zip -d /tmp/openni

# ──  Install & clean – reruns on changes only to this stanza ───────────
RUN set -e && \
    cd "$(find /tmp/openni -maxdepth 1 -type d -name 'OpenNI-Linux-x64*' | head -n1)" && \
    ./install.sh && \
    cp -f /usr/etc/udev/rules.d/*astra.rules /etc/udev/rules.d/ || true

# 4) Pull the ROS driver into your workspace and build it
RUN git clone --depth 1 https://github.com/orbbec/ros_astra_camera.git /catkin_ws/src/ros_astra_camera
#    (driver supports Noetic – see repo’s README) :contentReference[oaicite:0]{index=0}

# 5) Re-build workspace (adds astra_camera)
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
                  cd /catkin_ws && \
                  catkin_make -DCMAKE_BUILD_TYPE=Release"

#####################################################################


# ------------  Python user-level deps -----
RUN pip3 install --default-timeout=100 \
    numpy \
    scipy \
    matplotlib \
    boto3 \
    pyaudio \
    opencv-python \
    absl-py

RUN pip3 install --no-deps mediapipe==0.10.11 opencv-contrib-python

# ------------  Catkin workspace ----------
ENV CATKIN_WS=/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS

# Copy your code (instead of git-cloning inside the container)
#    Context path must include this Dockerfile and your repo
COPY . /$CATKIN_WS/src/ 

# copy the startup script
COPY ros_docker_auto_startup_launcher.sh /usr/local/bin/ros_docker_auto_startup_launcher.sh
RUN chmod +x /usr/local/bin/ros_docker_auto_startup_launcher.sh

# resolve rosdep and build
RUN rosdep init 

RUN . /opt/ros/noetic/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin build -DCMAKE_BUILD_TYPE=Release

# ------------  source overlays -----------
RUN echo "source /opt/ros/noetic/setup.bash"   >> /etc/bash.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /etc/bash.bashrc

# CMD ["bash"]

# set the default entrypoint (or you can use CMD instead)
ENTRYPOINT ["/usr/local/bin/ros_docker_auto_startup_launcher.sh"]
