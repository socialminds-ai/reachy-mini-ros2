# ROS 2 Lyrical container that runs ON the Reachy Mini (arm64, Raspberry Pi CM4).
#
# The robot's OS is Raspberry Pi OS / Debian trixie, for which there are no ROS 2
# Lyrical binaries, so ROS lives in this Ubuntu-based container. It is started by
# systemd (see reachy-mini-ros.service) with --network host and the servo serial
# port passed through, so nodes inside can reach both the Pollen daemon on
# localhost:8000 and, when the daemon is stopped, the motors directly; also with
# the camera devices and the host's PipeWire socket.
#
# Two variants, from the same layers up to the FULL argument:
#   - default (FULL=0): what the robot needs to run (about 1.6 GB). The robot's
#     SD card (14 GB) has room for one image only, so keep it small.
#   - FULL=1: adds build tools (colcon, gcc, the headers of the camera and
#     audio libraries), to build packages in /root/ros, and Pollen's Python SDK,
#     for the Python driver (`lang:=python`).
#
# Build (any machine with buildx, arm64 emulated):
#   docker buildx build --platform linux/arm64 -t reachy-mini-ros:lyrical docker/
#   docker buildx build --platform linux/arm64 --build-arg FULL=1 -t reachy-mini-ros:lyrical-full docker/
# or on the robot itself (slow, ~1 h):  docker build -t reachy-mini-ros:lyrical docker/
FROM ros:lyrical-ros-core
ARG ROS_DISTRO=lyrical
ENV DEBIAN_FRONTEND=noninteractive
USER root
# Some base image builds carry dpkg-statoverride entries for users that no
# longer exist (seen: 'geoclue'), which makes every later dpkg run abort.
RUN for u in $(awk '{print $1}' /var/lib/dpkg/statoverride 2>/dev/null | sort -u); do \
      getent passwd "$u" >/dev/null || sed -i "/^$u /d" /var/lib/dpkg/statoverride; done

# Upgrade first: packages installed below come from the latest ROS sync and can
# be ABI-incompatible with the older builds in the base image (seen: octomap_msgs
# 20260915 needing symbols missing from std_msgs 20260811 -> driver_node exit 127).
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
      curl gnupg ca-certificates iputils-ping iproute2 nano less \
      python3-requests python3-websockets python3-yaml \
      ros-${ROS_DISTRO}-rmw-zenoh-cpp \
      ros-${ROS_DISTRO}-demo-nodes-cpp ros-${ROS_DISTRO}-demo-nodes-py \
      ros-${ROS_DISTRO}-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# SocialMinds apt repository: the Reachy Mini driver packages come from there.
# Camera and audio: libcamerasrc and the Raspberry Pi IPA have no rosdep key, so
# reachy_mini_camera_driver cannot pull them in. pipewire-bin brings the client
# configuration that mic_frontend and pw_audio_play load; the PipeWire daemon
# itself runs on the host (README.md, section 1.4).
# No ffmpeg_image_transport: the camera driver publishes its packets without it,
# and it would bring OpenCV (~950 MB). Subscribers decode them on your machine.
RUN curl -fsSL https://socialminds.doc.iiia.csic.es/apt/public.key | gpg --dearmor -o /etc/apt/keyrings/socialminds.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/socialminds.gpg] https://socialminds.doc.iiia.csic.es/apt/ ${ROS_DISTRO} main" \
       > /etc/apt/sources.list.d/socialminds.list \
    && apt-get update && apt-get install -y --no-install-recommends \
      socialminds-ros-${ROS_DISTRO}-reachy-mini \
      gstreamer1.0-libcamera libcamera-ipa pipewire-bin \
    && rm -rf /var/lib/apt/lists/*

COPY zenoh_router.json5 /etc/reachy-mini-ros/zenoh_router.json5
COPY entrypoint.sh /usr/local/bin/reachy-mini-ros-entrypoint
RUN chmod +x /usr/local/bin/reachy-mini-ros-entrypoint
# Non-interactive shells (docker exec ... bash -c) get ROS too.
RUN printf 'source /opt/ros/%s/setup.bash\n[ -f /root/ros/install/setup.bash ] && source /root/ros/install/setup.bash\ntrue\n' "${ROS_DISTRO}" > /etc/reachy-mini-ros/env.sh
ENV BASH_ENV=/etc/reachy-mini-ros/env.sh
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV ZENOH_ROUTER_CONFIG_URI=/etc/reachy-mini-ros/zenoh_router.json5
RUN echo 'source /opt/ros/'${ROS_DISTRO}'/setup.bash' >> /root/.bashrc

ENTRYPOINT ["/usr/local/bin/reachy-mini-ros-entrypoint"]
CMD ["sleep", "infinity"]

# ---- FULL=1 only, below: the default image stops here (empty layers). ----
ARG FULL=0

# Building packages in /root/ros (README.md, section 2.4): colcon, the
# compilers, and the headers the camera driver, mic_frontend and pw_audio_play
# build against.
RUN if [ "$FULL" = 1 ]; then \
      apt-get update && apt-get install -y --no-install-recommends \
        build-essential git pkg-config python3-pip python3-dev \
        python3-colcon-common-extensions python3-rosdep \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
        libpipewire-0.3-dev libspeexdsp-dev libwebrtc-audio-processing-dev \
      && rm -rf /var/lib/apt/lists/*; \
    fi

# Pollen Robotics SDK, for the Python driver only (the C++ driver reads the
# emotions library straight from the mounted HuggingFace cache). Installed next
# to rclpy, see install_reachy_mini_sdk.sh. Rust only compiles some of its
# dependencies: it goes, with its crate cache (~500 MB), in the same layer.
COPY install_reachy_mini_sdk.sh /usr/local/bin/install_reachy_mini_sdk
RUN if [ "$FULL" = 1 ]; then \
      apt-get update && apt-get install -y --no-install-recommends \
        python3-gi python3-gst-1.0 gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0 libportaudio2 \
        cargo rustc \
      && chmod +x /usr/local/bin/install_reachy_mini_sdk && install_reachy_mini_sdk 1.10.0 \
      && apt-get purge -y --auto-remove cargo rustc \
      && rm -rf /var/lib/apt/lists/* /root/.cargo /root/.cache /root/.local; \
    fi
