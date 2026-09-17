# ROS 2 Lyrical container that runs ON the Reachy Mini (arm64, Raspberry Pi CM4).
#
# The robot's OS is Raspberry Pi OS / Debian trixie, for which there are no ROS 2
# Lyrical binaries, so ROS lives in this Ubuntu-based container. It is started by
# systemd (see reachy-mini-ros.service) with --network host and the servo serial
# port passed through, so nodes inside can reach both the Pollen daemon on
# localhost:8000 and, when the daemon is stopped, the motors directly.
#
# Build (any machine with buildx, arm64 emulated):
#   docker buildx build --platform linux/arm64 -t reachy-mini-ros:lyrical docker/
# or on the robot itself (slow, ~1 h):  docker build -t reachy-mini-ros:lyrical docker/
FROM ros:lyrical
ARG ROS_DISTRO=lyrical
ENV DEBIAN_FRONTEND=noninteractive
USER root
# Some base image builds carry dpkg-statoverride entries for users that no
# longer exist (seen: 'geoclue'), which makes every later dpkg run abort.
RUN for u in $(awk '{print $1}' /var/lib/dpkg/statoverride 2>/dev/null | sort -u); do \
      getent passwd "$u" >/dev/null || sed -i "/^$u /d" /var/lib/dpkg/statoverride; done

RUN apt-get update && apt-get install -y --no-install-recommends \
      curl gnupg ca-certificates iputils-ping iproute2 nano less \
      python3-pip python3-requests python3-websockets python3-yaml \
      ros-${ROS_DISTRO}-rmw-zenoh-cpp \
      ros-${ROS_DISTRO}-demo-nodes-cpp ros-${ROS_DISTRO}-demo-nodes-py \
      ros-${ROS_DISTRO}-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# SocialMinds apt repository: the Reachy Mini driver packages come from there
RUN curl -fsSL https://socialminds.doc.iiia.csic.es/apt/public.key | gpg --dearmor -o /etc/apt/keyrings/socialminds.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/socialminds.gpg] https://socialminds.doc.iiia.csic.es/apt/ ${ROS_DISTRO} main" \
       > /etc/apt/sources.list.d/socialminds.list \
    && apt-get update && apt-get install -y --no-install-recommends \
      socialminds-ros-${ROS_DISTRO}-reachy-mini \
    && rm -rf /var/lib/apt/lists/*

# Pollen Robotics SDK (optional: only the Python driver needs it; the C++ driver
# reads the emotions library straight from the mounted HuggingFace cache).
# Installed next to rclpy, see install_reachy_mini_sdk.sh.
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-gi python3-gst-1.0 gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0 libportaudio2 \
      cargo rustc python3-dev build-essential \
    && rm -rf /var/lib/apt/lists/*
COPY install_reachy_mini_sdk.sh /usr/local/bin/install_reachy_mini_sdk
RUN chmod +x /usr/local/bin/install_reachy_mini_sdk && install_reachy_mini_sdk 1.10.0

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
