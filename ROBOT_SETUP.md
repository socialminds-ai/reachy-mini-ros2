# Preparing a Reachy Mini for ROS 2

This guide takes an off-the-shelf **Reachy Mini wireless** (Raspberry Pi CM4
inside, Raspberry Pi OS / Debian 13 "trixie", arm64) from Pollen's stock setup
to a robot that runs ROS 2 Lyrical nodes, either next to the Pollen daemon or
instead of it. Everything happens over ssh; nothing is flashed. Allow 20 minutes
plus the image download.

Why a container: Debian trixie has no ROS 2 Lyrical binaries, so ROS lives in an
Ubuntu-based Docker image (`docker/Dockerfile`) that runs on the robot with
host networking and the servo serial port passed through.

## 0. What you need

* The robot powered, on the same WiFi as your laptop, onboarded once with
  Pollen's app (so it has a hostname and joins the network).
* Its hostname or IP: `reachy-mini.local` for a single robot, otherwise
  `reachy-mini-XXXX.local` (mDNS), or the IP shown by the Pollen app or your router.
* The `pollen` account on the robot, with its password (the one set at
  onboarding; `root` on the lab's robots) and `sudo`.
* A laptop with `ssh`. Docker on the laptop is not needed.

## 1. Log in and fix the clock

```sh
ssh pollen@reachy-mini.local
date -u
```

If the date is wrong (it drifts after a long power-off), `apt` will refuse the
repositories with "Not live until ...". Set it from the laptop's clock, then let
NTP take over:

```sh
# on the laptop
ssh -t pollen@reachy-mini.local "sudo date -u -s '$(date -u)'"
# on the robot
sudo systemctl restart systemd-timesyncd
timedatectl | grep synchronized
```

## 2. Install Docker

```sh
sudo apt update
sudo apt install -y docker.io
sudo systemctl enable --now docker
sudo usermod -aG docker pollen
```

Log out and in again so that `docker ps` works without `sudo`.

## 3. Get the ROS 2 image

Two options.

**Pull it** (minutes). The image is built by this repository's `robot-image`
pipeline job and published in the lab's GitLab registry:

```sh
docker login gitlab.iiia.csic.es:5005          # your GitLab credentials or a deploy token
docker pull gitlab.iiia.csic.es:5005/socialminds/robots/reachymini/reachy-mini-driver/reachy-mini-ros:lyrical
docker tag gitlab.iiia.csic.es:5005/socialminds/robots/reachymini/reachy-mini-driver/reachy-mini-ros:lyrical reachy-mini-ros:lyrical
```

**Or build it on the robot** (about an hour on the Pi, needs Internet):

```sh
git clone https://gitlab.iiia.csic.es/socialminds/robots/reachymini/reachy-mini-driver.git
docker build -t reachy-mini-ros:lyrical reachy-mini-driver/docker/
```

What is inside: ROS 2 Lyrical (`ros:lyrical`, Ubuntu 26.04), `rmw_zenoh_cpp`
with a router configured to listen on port 7447, `robot_state_publisher`, the
demo nodes, the lab's apt repository with the `reachy_mini` driver packages
already installed, and Pollen's Python SDK (for the optional Python driver).

## 4. Keep the container running (systemd)

`docker/reachy-mini-ros.service` starts the container at boot with the right
options. It must keep the servo port and the daemon's move cache visible:

* `--network host`: nodes and the zenoh router use the robot's own IP;
  the Pollen daemon on `localhost:8000` stays reachable from inside;
* `--device /dev/ttyAMA3`: the servo bus (only usable while the daemon is stopped);
* `/home/pollen/.cache/huggingface` read-only: the emotions library the daemon
  downloaded, which the driver plays;
* `/home/pollen/ros` at `/root/ros`: a scratch workspace for packages built from source.

```sh
# from the cloned repository, or copy the two files over with scp
sudo cp reachy-mini-driver/docker/reachy-mini-ros.service /etc/systemd/system/
mkdir -p ~/ros
sudo systemctl daemon-reload
sudo systemctl enable --now reachy-mini-ros
systemctl status reachy-mini-ros
```

`docker/provision.sh` does steps 1 to 4 in one go (`sudo bash provision.sh`); it
pulls the image if you are logged in to the registry, and builds it otherwise.

Check:

```sh
docker exec reachy-mini-ros ros2 pkg list | grep reachy_mini
docker exec -it reachy-mini-ros bash          # a shell with ROS sourced
```

At this point the Pollen daemon still owns the robot and the container only
observes: this is fine for nodes that talk to the daemon's HTTP API on port 8000.

## 5. Reach the robot from a laptop (zenoh)

Nodes on the laptop must use `rmw_zenoh_cpp` too, with a router that connects
to the robot's router on TCP port 7447 (no multicast discovery is involved, so
this also works from a container or across a NAT):

```sh
# on the laptop, ROS 2 Lyrical with ros-lyrical-rmw-zenoh-cpp installed
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/<robot IP>:7447"]' ros2 run rmw_zenoh_cpp rmw_zenohd &
ros2 node list
```

Use the IP rather than the `.local` name if mDNS does not resolve where the
command runs (containers usually cannot).

## 6. Run the ROS driver instead of the daemon

The driver and the daemon cannot share the servo bus. To try the driver by hand:

```sh
sudo systemctl stop reachy-mini-daemon
docker exec -it reachy-mini-ros ros2 launch reachy_mini driver.launch.py
```

In another terminal (robot or laptop): `ros2 topic echo /joint_states`,
`ros2 action send_goal /skill/replay_motion motions_skills/action/ReplayMotion "{name: cheerful1}"`
(the driver implements the ROS4HRI motion and `look_at` skills, see `README.md`).
Stop with Ctrl-C: the driver puts the head in the sleep pose and releases the
motors. Then `sudo systemctl start reachy-mini-daemon` gives the robot back to
Pollen's stack.

To make the driver the default at boot, install the second unit. It conflicts
with the daemon's unit, so systemd never runs both, and it stops the driver
gently (SIGINT inside the container) so the sleep sequence runs:

```sh
sudo cp reachy-mini-driver/docker/reachy-mini-driver-docker.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl disable reachy-mini-daemon        # otherwise the two race at boot
sudo systemctl enable --now reachy-mini-driver-docker
journalctl -u reachy-mini-driver-docker -f
```

What you lose without the daemon: sound, camera/WebRTC, the HTTP API on port
8000 and the Pollen apps. What you keep: the whole ROS graph, `joint_states`,
`/tf` with the robot model, the ROS4HRI skills.

## 7. Updating

```sh
docker exec reachy-mini-ros apt update
docker exec reachy-mini-ros apt install -y socialminds-ros-lyrical-reachy-mini   # newer driver packages
```

Note that the container is started with `--rm`: anything installed by hand inside
it is lost when the unit restarts it. For a permanent change, rebuild or re-pull
the image (`docker pull ...` then `sudo systemctl restart reachy-mini-ros`).

## 8. Back to stock

```sh
sudo systemctl disable --now reachy-mini-driver-docker   # if installed
sudo systemctl enable --now reachy-mini-daemon
sudo systemctl disable --now reachy-mini-ros             # optional: stop the container too
```

Docker and the image can stay; they do not interfere with the daemon.

## Troubleshooting

`apt` says "Not live until ..." or signature errors
: The clock. Step 1.

`docker pull` says "unauthorized"
: The registry needs a login (step 3); or build the image on the robot.

The driver logs "cannot open /dev/ttyAMA3"
: The Pollen daemon still owns the port: `sudo systemctl stop reachy-mini-daemon`.
  `sudo fuser /dev/ttyAMA3` shows who has it.

The laptop sees no nodes
: Router not connected: check `ZENOH_CONFIG_OVERRIDE` and that TCP 7447 is
  reachable (`nc -vz <robot IP> 7447`); both sides must use `rmw_zenoh_cpp`.

`ros2` not found in `docker exec`
: Use `docker exec reachy-mini-ros bash -c "..."` or a plain `ros2 ...` command:
  the image sources ROS for every bash, including non-interactive ones.
