# -----------------------------------------------------------------------------
# FloSystemV2 – ROS Noetic runtime image
# -----------------------------------------------------------------------------
# * Based on the minimal ros:noetic-ros-core image
# * Installs only the extra system & Python packages actually required
# * Resolves rosdep keys that previously failed (python‑serial, rgbd_launch)
# * Skips the python3‑mediapipe key because we install it with pip instead
# * Uses **catkin_tools** from the outset (no catkin_make artefacts lying around)
# * Builds the workspace in /catkin_ws and exposes it through /etc/bash.bashrc
# -----------------------------------------------------------------------------

FROM ros:noetic-ros-core

LABEL maintainer="FloSystem team <flo@flo.ai>"
ARG DEBIAN_FRONTEND=noninteractive

# -----------------------------------------------------------------------------
# 1️⃣ System‑level dependencies
# -----------------------------------------------------------------------------
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    # ── Build / dev tools ────────────────────────────────────────────────────
        build-essential cmake git curl wget \
    # ── ROS tooling ─────────────────────────────────────────────────────────
        python3-rosdep python3-catkin-tools python3-rosinstall \
        python3-rosinstall-generator python3-wstool \
        python3-pip python3-setuptools \
    # ── Packages missing from rosdep keys ───────────────────────────────────
        python3-serial \   
        ros-noetic-rgbd-launch \  
    # ── Misc – clean up afterwards ──────────────────────────────────────────
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# 2️⃣ Python‑level dependencies
# -----------------------------------------------------------------------------
RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir \
        numpy scipy scikit-learn \
        opencv-contrib-python\
        mediapipe==0.10.11 \
        pyserial pyyaml

# -----------------------------------------------------------------------------
# 3️⃣ (Optional) Orbbec OpenNI 2 SDK & ROS driver for Astra camera
#      ▸ keep these in a single layer so they can be skipped easily if not
#        required. Feel free to comment the whole block out if you are building
#        on a machine that does not need Astra depth cameras.
# -----------------------------------------------------------------------------
ENV ORBBEC_OPENNI_URL="https://dl.orbbec3d.com/openni/OpenNI_2.3.0.1033_arm64.deb"
RUN curl -L "$ORBBEC_OPENNI_URL" -o /tmp/orbbec_openni.deb && \
    apt-get update && apt-get install -y /tmp/orbbec_openni.deb && \
    rm /tmp/orbbec_openni.deb && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 https://github.com/orbbec/ros_astra_camera /tmp/astra_camera && \
    mv /tmp/astra_camera /opt/astra_camera

# -----------------------------------------------------------------------------
# 4️⃣ Create & populate the catkin workspace
# -----------------------------------------------------------------------------
ENV ROS_WS=/catkin_ws
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# Copy the repository *after* installing the OS/Py deps so that code changes
# don’t invalidate the heavier dependency layers.
COPY . ${ROS_WS}/src/

# -----------------------------------------------------------------------------
# 5️⃣ rosdep – initialise once & install package dependencies
#      ▸ python‑serial & rgbd_launch are now satisfied via apt. We still need
#        to skip python3‑mediapipe because that dep is pip‑installed above.
# -----------------------------------------------------------------------------
RUN rosdep init && \
    rosdep fix-permissions && \
    rosdep update && \
    . /opt/ros/noetic/setup.sh && \
    rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y \
        --skip-keys="python3-mediapipe"

# -----------------------------------------------------------------------------
# 6️⃣ Build the workspace (Release mode, installed layout)
# -----------------------------------------------------------------------------
RUN . /opt/ros/noetic/setup.sh && \
    rm -rf build devel || true && \
    catkin config --install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build

# Source the workspace for all future shells
RUN echo "source /catkin_ws/install/setup.bash" >> /etc/bash.bashrc

# -----------------------------------------------------------------------------
# 7️⃣ Runtime entrypoint & default command
# -----------------------------------------------------------------------------
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
