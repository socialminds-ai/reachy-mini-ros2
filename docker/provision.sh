#!/bin/bash
# One-shot provisioning of a Reachy Mini (wireless) for ROS 2.
# Run it ON the robot:   sudo bash provision.sh
# Details and the manual steps: ../README.md (section 1, Installation)
#
# What it does (idempotent):
#   1. points timesyncd at a reachable NTP server and waits for sync
#      (apt refuses signatures "from the future"; zenoh rejects timestamps
#      more than 500 ms off the laptop's clock)
#   2. installs Docker and lets `pollen` use it
#   3. pulls the reachy-mini-ros image (or builds it locally, ~1 h)
#   4. installs and starts the systemd unit that keeps the container running
# It does NOT touch the Pollen daemon: the ROS container coexists with it.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
IMAGE=${IMAGE:-reachy-mini-ros:lyrical}
REGISTRY_IMAGE=${REGISTRY_IMAGE:-gitlab.iiia.csic.es:5005/socialminds/robots/reachymini/reachy-mini-driver/reachy-mini-ros:lyrical}
# The Debian pool is blocked on some networks (the lab's included)
NTP_SERVER=${NTP_SERVER:-hora.roa.es}

[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }

echo "== 1. clock (NTP: $NTP_SERVER)"
mkdir -p /etc/systemd/timesyncd.conf.d
printf '[Time]\nNTP=%s\nFallbackNTP=0.debian.pool.ntp.org 1.debian.pool.ntp.org\n' "$NTP_SERVER" \
  > /etc/systemd/timesyncd.conf.d/lab.conf
systemctl restart systemd-timesyncd
for _ in $(seq 15); do
  timedatectl show -p NTPSynchronized --value | grep -q yes && break
  sleep 1
done
if timedatectl show -p NTPSynchronized --value | grep -q yes; then
  echo "clock synchronised"
else
  echo "WARNING: clock not synchronised; set it by hand (see README.md, section 1.2)"
fi
date -u

echo "== 2. docker"
if ! command -v docker >/dev/null; then
  apt-get update -qq
  apt-get install -y -qq docker.io
fi
systemctl enable --now docker
usermod -aG docker pollen

echo "== 3. image"
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  if docker pull "$REGISTRY_IMAGE" 2>/dev/null; then
    docker tag "$REGISTRY_IMAGE" "$IMAGE"
  else
    echo "registry image not available (not logged in?), building locally (about an hour on a Pi)"
    docker build -t "$IMAGE" "$HERE"
  fi
fi

echo "== 4. systemd unit"
mkdir -p /home/pollen/ros && chown pollen:pollen /home/pollen/ros
install -m 644 "$HERE/reachy-mini-ros.service" /etc/systemd/system/reachy-mini-ros.service
systemctl daemon-reload
systemctl enable --now reachy-mini-ros
sleep 3
systemctl --no-pager --lines=3 status reachy-mini-ros | head -5

echo
echo "Done. Try:   docker exec -it reachy-mini-ros bash"
