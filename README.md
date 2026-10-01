# Reachy Mini ROS 2

ROS 2 Lyrical support for the Pollen Robotics Reachy Mini (wireless): the
servos, the robot model and TF, the camera, the microphones and the speaker, and
the [ROS4HRI skills](https://ros4hri.github.io/skills/) (`replay_motion`,
`look_at`, ...), all as ROS nodes that run on the robot and own the hardware
directly, without the Pollen daemon.

This page follows the main path:

1. **[Installation](#1-installation-turn-a-reachy-mini-into-a-ros-robot)**: take
   an off-the-shelf Reachy Mini and make it a ROS robot, keeping the Pollen
   daemon as the default or making ROS the default;
2. **[Usage](#2-usage)**: start ROS on the robot, connect your own machine to it
   over zenoh, open RViz;
3. **[Tests](#3-tests-is-everything-working)**: check in a few minutes that
   everything works.

The interface of the nodes (parameters, topics, skills, TF), the camera and audio
details, native and from-source installations are in
**[REFERENCE.md](REFERENCE.md)**; why the driver is built the way it is, in
**[DESIGN.md](DESIGN.md)**.

## How the pieces fit

![The ROS container on the robot, the Pollen daemon, the hardware they share, and your machine connected over zenoh](doc/architecture.svg)

The robot runs Debian trixie, which has no ROS 2 Lyrical binaries, so ROS runs
there in an Ubuntu-based Docker container (image built from
[`docker/Dockerfile`](docker/Dockerfile)). All ROS traffic uses
`rmw_zenoh_cpp`: the container runs a zenoh router on port 7447, and your machine
runs a second router that connects to it (no multicast, so it works from a
container or through a NAT too).

Only one process can own the servos and the camera: either the Pollen daemon
(Pollen's apps, HTTP API, WebRTC) or the ROS driver. The installation lets you
pick which one starts at boot.

## 1. Installation: turn a Reachy Mini into a ROS robot

Everything happens over ssh; nothing is flashed. Allow 20 minutes plus the image
download.

### 1.1 What you need

* The robot powered, on the same network as your machine, onboarded once with
  Pollen's app (so it has a hostname and joins the WiFi).
* Its hostname or IP: `reachy-mini.local` for a single robot, otherwise
  `reachy-mini-XXXX.local` (mDNS), or the IP shown by the Pollen app or your
  router.
* The `pollen` account on the robot, which has `sudo`. Its default password
  is `root`.

### 1.2 Log in, fix the clock, get this repository

```sh
ssh pollen@reachy-mini.local
date -u
```

The robot has no battery-backed clock, so the date drifts after a long power-off
and `apt` then refuses the repositories ("Not live until ..."). The clock must
also agree with your machine's to within 500 ms, otherwise zenoh rejects the
robot's messages.

The stock NTP servers (the Debian pool) are blocked on some networks, and
`timesyncd` then never synchronises. Point it at a server that answers, here
`hora.roa.es` (Spain's official time):

```sh
# on the robot
sudo mkdir -p /etc/systemd/timesyncd.conf.d
printf '[Time]\nNTP=hora.roa.es\nFallbackNTP=0.debian.pool.ntp.org 1.debian.pool.ntp.org\n' \
  | sudo tee /etc/systemd/timesyncd.conf.d/lab.conf
sudo systemctl restart systemd-timesyncd
timedatectl | grep synchronized          # must say "yes" after a few seconds
```

Your own machine's clock must be right too (a container uses its host's clock).
If no NTP server is reachable at all, copy your machine's clock as a stopgap
(accurate to about a second only):
`ssh -t pollen@reachy-mini.local "sudo date -u -s '$(date -u)'"`.

Then clone this repository on the robot; the rest of the installation uses the
files of its `docker/` directory:

```sh
git clone https://gitlab.iiia.csic.es/socialminds/robots/reachymini/reachy-mini-driver.git
```

### 1.3 Docker, the ROS image and the container

Three steps: install Docker, get the ROS image, and have systemd keep the
container running.

> 💡 **Tip: one script instead of the steps below.**
> `docker/provision.sh` does all of this section, and the clock setup of
> section 1.2, in one go (it is idempotent). Run it *instead of* the manual
> steps below, not before them:
>
> ```sh
> sudo bash reachy-mini-driver/docker/provision.sh
> ```
>
> It pulls the pre-built image from the IIIA registry if root is logged in to
> it (install Docker and run `sudo docker login gitlab.iiia.csic.es:5005`
> first), and otherwise builds it on the robot (about an hour). Then go on to
> section 1.4.

**Docker**:

```sh
sudo apt update
sudo apt install -y docker.io
sudo systemctl enable --now docker
sudo usermod -aG docker pollen          # then log out and in again
```

**The image**, either pulled or built.

The pre-built image (built by this repository's `robot-image` pipeline job) is
in the IIIA GitLab registry, which is currently only open to IIIA staff; this
may change. Pulling it takes minutes:

```sh
docker login gitlab.iiia.csic.es:5005          # your GitLab credentials or a deploy token
docker pull gitlab.iiia.csic.es:5005/socialminds/robots/reachymini/reachy-mini-driver/reachy-mini-ros:lyrical
docker tag gitlab.iiia.csic.es:5005/socialminds/robots/reachymini/reachy-mini-driver/reachy-mini-ros:lyrical reachy-mini-ros:lyrical
```

Anyone can instead build the same image on the robot from this repository
(about an hour, needs Internet):

```sh
docker build -t reachy-mini-ros:lyrical reachy-mini-driver/docker/
```

It contains ROS 2 Lyrical (Ubuntu 26.04), `rmw_zenoh_cpp` with a router
listening on port 7447, the `reachy_mini` packages of this repository and the
audio nodes from the lab's apt repository, and Pollen's Python SDK (for the
optional Python driver).

**The container**, kept running by systemd (`docker/reachy-mini-ros.service`):

```sh
sudo cp reachy-mini-driver/docker/reachy-mini-ros.service /etc/systemd/system/
mkdir -p ~/ros
sudo systemctl daemon-reload
sudo systemctl enable --now reachy-mini-ros
```

The unit starts the container with `--network host` (the zenoh router and the
nodes use the robot's IP; the Pollen daemon's API on `localhost:8000` stays
reachable), the servo port `/dev/ttyAMA3`, the camera devices (`--privileged`,
`/run/udev`), PipeWire's socket (`/run/user/1000`), the daemon's cache of the
emotions library (read-only), and `~/ros` as `/root/ros`, a workspace for
packages you build from source.

The container only starts nodes when asked to (section 2.1). In it, `ros2` must
always go through `bash`, which sources ROS:
`docker exec reachy-mini-ros bash -c "ros2 ..."`, or a shell with
`docker exec -it reachy-mini-ros bash`.

### 1.4 Audio: PipeWire

The microphones and the speaker go through PipeWire, which Raspberry Pi OS
already has but only starts when someone logs in. Make it start at boot and give
it the robot's two audio nodes. They open the card through the same ALSA sharing
devices as the daemon, so audio works whether the daemon runs or not.

```sh
sudo loginctl enable-linger pollen
mkdir -p ~/.config/pipewire/pipewire.conf.d ~/.config/wireplumber/wireplumber.conf.d
cp reachy-mini-driver/docker/pipewire/reachy-mini-audio.conf ~/.config/pipewire/pipewire.conf.d/
cp reachy-mini-driver/docker/pipewire/51-reachy-mini.conf ~/.config/wireplumber/wireplumber.conf.d/
systemctl --user restart pipewire wireplumber pipewire-pulse
wpctl status      # Sinks: "Reachy Mini speaker", Sources: "Reachy Mini microphones"
sudo systemctl restart reachy-mini-ros     # so the container sees PipeWire's socket
```

`51-reachy-mini.conf` keeps WirePlumber off the audio card (PipeWire uses it
through the sharing devices only) and off the video devices (a PipeWire camera
monitor would hold the camera).

### 1.5 Choose what owns the robot at boot

**Option A: keep the Pollen daemon as the default.** Nothing more to do. The
robot boots as Pollen ships it (apps, HTTP API, WebRTC); the ROS container runs
next to the daemon, and you hand the robot over to ROS when you need it
(section 2.1). Choose this if the robot is also used with Pollen's tools.

**Option B: make ROS the default.** The robot boots straight into the ROS
driver (servos, model, camera, audio) and the daemon no longer starts. The
`docker/reachy-mini-driver-docker.service` unit runs
`ros2 launch reachy_mini driver.launch.py` in the container; it declares
`Conflicts=reachy-mini-daemon.service`, so systemd never runs both, and stops the
driver with SIGINT so that the robot goes to its sleep pose and releases the
motors.

```sh
sudo cp reachy-mini-driver/docker/reachy-mini-driver-docker.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl disable reachy-mini-daemon        # otherwise the two race at boot
sudo systemctl enable --now reachy-mini-driver-docker
```

What you lose without the daemon: its HTTP API on port 8000, WebRTC and the
Pollen apps. What you get instead: the whole robot in ROS (`joint_states`, `/tf`
with the robot model, the camera, the microphones and the speaker, the ROS4HRI
skills).

To switch from B back to A:

```sh
sudo systemctl disable --now reachy-mini-driver-docker
sudo systemctl enable --now reachy-mini-daemon
```

### 1.6 Updating, and going back to stock

The newest driver packages come with a newer image: pull it from the registry,
or rebuild it, then remove the previous image and restart the container.

With access to the registry:

```sh
docker pull gitlab.iiia.csic.es:5005/socialminds/robots/reachymini/reachy-mini-driver/reachy-mini-ros:lyrical
docker tag gitlab.iiia.csic.es:5005/socialminds/robots/reachymini/reachy-mini-driver/reachy-mini-ros:lyrical reachy-mini-ros:lyrical
```

Otherwise, rebuild it (`--no-cache` matters: without it Docker reuses the cached
`apt install` layers, with the old packages):

```sh
git -C reachy-mini-driver pull
docker build --no-cache -t reachy-mini-ros:lyrical reachy-mini-driver/docker/
```

Then, in both cases:

```sh
docker image prune -f                  # remove the previous image
docker builder prune -f                # and the build cache, after a rebuild
sudo systemctl restart reachy-mini-ros
```

Do not skip the prune: the previous image stays on the card, untagged, after a
pull or a rebuild. The robot's root partition is about 14 GB and an image about
4 GB, so two or three of them fill it up, and then the container can no longer
start. `docker images` should list a single `reachy-mini-ros` image (under two
names if pulled) and `docker system df` about 4 GB. If the card is already too
full for the update, remove the current image first, then update and restart as
above:

```sh
sudo systemctl stop reachy-mini-ros
docker rmi reachy-mini-ros:lyrical
docker rmi gitlab.iiia.csic.es:5005/socialminds/robots/reachymini/reachy-mini-driver/reachy-mini-ros:lyrical   # if pulled
docker image prune -f
docker builder prune -f
```

`docker exec reachy-mini-ros bash -c "apt update && apt install -y socialminds-ros-lyrical-reachy-mini"`
updates them in place, but only until the next restart of the container, which
is started with `--rm`: anything installed by hand in it is lost.

Back to the stock robot: switch to option A (above), then optionally
`sudo systemctl disable --now reachy-mini-ros` to stop the container. Docker and
the image can stay; they do not interfere with the daemon.

## 2. Usage

### 2.1 Start ROS on the robot

**Option A (daemon by default)**: stop the daemon, then launch the robot in the
container:

```sh
# on the robot
sudo systemctl stop reachy-mini-daemon
docker exec -it reachy-mini-ros bash -c "ros2 launch reachy_mini driver.launch.py"
```

Ctrl-C stops it: the head goes to the sleep pose and the motors are released.
`sudo systemctl start reachy-mini-daemon` then gives the robot back to Pollen's
stack.

**Option B (ROS by default)**: the driver already runs.
`journalctl -u reachy-mini-driver-docker -f` shows its log;
`sudo systemctl restart reachy-mini-driver-docker` restarts it.

`driver.launch.py` starts the whole robot: the servo driver (C++),
`robot_state_publisher` with the robot model, the camera and the audio nodes.
Its arguments: `lang:=python` for the Python driver, `camera:=false`,
`audio:=false`, `description:=false` to leave parts out, `serial_port:=fake` for a
simulated servo bus, `params_file:=...` for other driver parameters (see
REFERENCE.md).

With the daemon running, ROS can still use the audio, and any node can talk to
the daemon's HTTP API on `localhost:8000`, but neither the servo driver nor the
camera can start:

```sh
docker exec -it reachy-mini-ros bash -c "ros2 launch mic_frontend mic_frontend.launch.py"
docker exec -it reachy-mini-ros bash -c "ros2 launch pw_audio_play pw_audio_play.launch.py"
```

### 2.2 Connect your machine to the robot (zenoh)

Your machine needs ROS 2 Lyrical with `rmw_zenoh_cpp`, RViz, the
`ffmpeg_image_transport` plugin (RViz decodes the camera's H.264 with it) and the
robot's description package (RViz loads the robot's meshes from it). Its clock must be synchronised (section 1.2).

**Natively (Ubuntu 26.04)**, with ROS 2 Lyrical installed:

```sh
# once: the SocialMinds apt repository
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://socialminds.doc.iiia.csic.es/apt/public.key | gpg --dearmor | sudo tee /etc/apt/keyrings/socialminds.gpg > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/socialminds.gpg] https://socialminds.doc.iiia.csic.es/apt/ lyrical main" | sudo tee /etc/apt/sources.list.d/socialminds.list
sudo apt update
sudo apt install ros-lyrical-rmw-zenoh-cpp ros-lyrical-rviz2 ros-lyrical-ffmpeg-image-transport \
                 socialminds-ros-lyrical-reachy-mini-description
```

The full metapackage, `socialminds-ros-lyrical-reachy-mini`, is not needed on
your machine: it only adds the drivers.

**In a Docker container** (any Linux host), with the host's display for RViz:

```sh
xhost +local:docker                    # once per session: let the container open windows
docker run -it --name ros-lyrical --network host \
    -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
    ros:lyrical bash
# then, inside: apt update && apt install -y curl gnupg,
# and the same apt commands as above (without sudo)
```

`--network host` is the simplest; the container then shares the host's port 7447,
so do not also run a zenoh router on the host. `docker start -ai ros-lyrical`
gets back into it later with the packages still installed. On a GPU-less setup,
`-e LIBGL_ALWAYS_SOFTWARE=1` helps RViz start.

**Then, in every terminal** (native or in the container), set the middleware and,
once per machine, start a zenoh router that connects to the robot's:

```sh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/<robot IP>:7447"]' ros2 run rmw_zenoh_cpp rmw_zenohd &
ros2 node list
```

Use the robot's IP rather than its `.local` name wherever mDNS does not resolve
(containers usually cannot). The router is needed once per machine (or
container), but `RMW_IMPLEMENTATION` must be set in every terminal: a `ros2`
command without it uses the default middleware and sees nothing of the robot.

**Tip.** This function, added to your `~/.bashrc`, does both: the first call
starts the router (after checking that the robot answers), later calls in other
terminals only set `RMW_IMPLEMENTATION`.

```sh
# Set up this terminal to talk to the Reachy Mini over zenoh.
# Usage: zenoh_reachy [robot IP]   (default: $REACHY_IP)
zenoh_reachy() {
  local ip="${1:-$REACHY_IP}"
  local log=/tmp/rmw_zenohd.log
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  if (: </dev/tcp/127.0.0.1/7447) 2>/dev/null; then
    echo "zenoh router already running (log: $log)"
    return 0
  fi
  if [ -z "$ip" ]; then
    echo "usage: zenoh_reachy <robot IP>   (or set REACHY_IP)"
    return 1
  fi
  if ! timeout 2 bash -c ": </dev/tcp/$ip/7447" 2>/dev/null; then
    echo "ERROR: no zenoh router at $ip:7447 (wrong IP? robot container down?)"
    return 1
  fi
  # in a subshell: detached from this terminal, and no job-control noise
  (ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"tcp/$ip:7447\"]" \
    setsid ros2 run rmw_zenoh_cpp rmw_zenohd >"$log" 2>&1 </dev/null &)
  local i
  for i in $(seq 20); do
    if (: </dev/tcp/127.0.0.1/7447) 2>/dev/null; then
      ros2 daemon stop >/dev/null 2>&1   # a daemon started with another RMW would be stale
      echo "zenoh router started, connected to $ip (log: $log)"
      return 0
    fi
    sleep 0.25
  done
  echo "ERROR: zenoh router did not start, see $log:"
  tail -5 "$log"
  return 1
}
```

The router keeps running when its terminal closes. If it was started with the
wrong IP, `pkill rmw_zenohd` and call `zenoh_reachy <robot IP>` again.

### 2.3 RViz

```sh
zenoh_reachy <robot IP>        # or the two commands above
rviz2 -d $(ros2 pkg prefix reachy_mini_description)/share/reachy_mini_description/rviz/reachy_mini.rviz
```

The configuration shows the robot model (from `/robot_description`), the TF
frames and the camera image (`/camera/image_raw/ffmpeg`). The camera starts when
RViz subscribes to it, so the image appears after a second or two.

### 2.4 Your own nodes

Nodes on your machine see the robot's topics, services and actions as if they
were local, as long as they run with `RMW_IMPLEMENTATION=rmw_zenoh_cpp` next to
the router. The driver's interface (topics, the ROS4HRI skills, TF frames) and
the camera and audio topics are listed in [REFERENCE.md](REFERENCE.md).

Nodes that must run on the robot (raw camera frames, low latency) go in the
container: build them in `~/ros` on the robot (`/root/ros` in the container,
which sources `/root/ros/install/setup.bash` when it exists):

```sh
docker exec -it reachy-mini-ros bash
cd /root/ros && colcon build
```

## 3. Tests: is everything working?

From your machine, in a terminal set up as in section 2.2, with ROS started on
the robot (section 2.1). Each check names what to look at if it fails
([Troubleshooting](#troubleshooting)).

| check | command | expected |
|---|---|---|
| the robot's router answers | `nc -vz <robot IP> 7447` | "succeeded" / "open" |
| the graph is visible | `ros2 node list` | `/reachy_mini_driver`, `/robot_state_publisher`, `/camera/camera`, the audio nodes |
| servos | `ros2 topic hz /joint_states` | about 50 Hz |
| kinematics and TF | `ros2 run tf2_ros tf2_echo base_link head` | the head about 0.18 m above `base_link` |
| motion skills | `ros2 service call /skill/list_motions motions_skills/srv/ListMotions` | the names of the emotions library |
| | `ros2 action send_goal /skill/replay_motion motions_skills/action/ReplayMotion "{name: cheerful1}"` | the robot plays the move; the goal succeeds |
| | `ros2 action send_goal /skill/look_at interaction_skills/action/LookAt "{policy: glance, target: {header: {frame_id: base_link}, point: {x: 1.0, y: 0.5, z: 0.3}}}"` | the head glances to its left for 2 s and comes back |
| camera | `ros2 topic hz /camera/image_raw/ffmpeg` | about 30 Hz (after a second: the camera starts on the first subscriber) |
| microphones | `ros2 topic hz /audio/head/raw` | about 100 Hz (10 ms chunks) |
| | `ros2 topic echo /audio/head/voice_detected` | `true` while someone speaks near the robot |
| everything together | RViz, section 2.3 | the model moves with the robot, the camera image shows |

The motion checks move the robot: make sure it has room.

On the robot itself, `systemctl status reachy-mini-ros` (and
`reachy-mini-driver-docker` in option B) must say "active (running)",
`docker exec reachy-mini-ros bash -c "ros2 pkg list" | grep reachy_mini` must list
the packages, and `wpctl status` must list the Reachy Mini speaker and
microphones.

**Without a robot**, on your machine, the whole stack runs on a simulated servo
bus. In a terminal without the zenoh setup of section 2.2 (or with a local
router: `ros2 run rmw_zenoh_cpp rmw_zenohd &`):

```sh
ros2 launch reachy_mini driver.launch.py serial_port:=fake camera:=false audio:=false rviz:=true
```

RViz shows the model; the skills above work and move it.

**Unit tests**, from source (see REFERENCE.md for the build): the drivers'
tests run on the fake bus.

```sh
colcon test && colcon test-result --verbose
```

## Troubleshooting

`apt` says "Not live until ..." or signature errors
: The clock. Section 1.2.

zenoh logs "incoming timestamp ... exceeding delta 500ms is rejected"
: The robot's and your machine's clocks differ by more than 500 ms. Check
  `timedatectl | grep synchronized` on both and set up NTP as in section 1.2.

`docker pull` says "unauthorized"
: The registry needs a login, and is currently only open to IIIA staff: build
  the image on the robot instead (section 1.3).

"no space left on device", or `df -h /` on the robot shows the card full
: Old images left behind by updates: `docker system df` shows their size, and
  `docker image prune -f` removes the untagged ones (section 1.6).

Your machine sees no nodes
: Router not connected: check `ZENOH_CONFIG_OVERRIDE` and that TCP 7447 is
  reachable (`nc -vz <robot IP> 7447`); both sides must use `rmw_zenoh_cpp`.
  The router's startup log does not say whether the connection succeeded, so a
  wrong IP goes unnoticed (`zenoh_reachy` checks it).

`ros2 action send_goal` stays at "Waiting for an action server to become available..." (or `ros2 node list` is empty) in one terminal but not in another
: `RMW_IMPLEMENTATION` is not set in that terminal, so `ros2` uses the default
  middleware. `export RMW_IMPLEMENTATION=rmw_zenoh_cpp` (or `zenoh_reachy`),
  then `ros2 daemon stop` so that a daemon started without it is not reused.

RViz shows the robot model in white, or no model
: `socialminds-ros-lyrical-reachy-mini-description` is not installed on your machine (RViz resolves
  the meshes' `package://` paths locally), or `robot_state_publisher` is not
  running on the robot (`description:=false`).

RViz shows no camera image
: `ros-lyrical-ffmpeg-image-transport` is missing on your machine, or the camera
  driver is not running (see below).

The driver logs "cannot open /dev/ttyAMA3"
: The Pollen daemon still owns the port: `sudo systemctl stop reachy-mini-daemon`.
  `sudo fuser /dev/ttyAMA3` shows who has it.

The camera driver logs "cannot start the camera" and retries
: Another process has the camera, usually the Pollen daemon:
  `sudo systemctl stop reachy-mini-daemon`. If `rpicam-hello --list-cameras` lists
  no camera even then, check the kernel: `dmesg | grep imx708` must not show
  "failed to read chip id" for the camera's port (a loose or reversed CSI ribbon
  gives exactly that; the other port always fails, there being one camera).

`wpctl status` lists no Reachy Mini source or sink
: The configuration files of section 1.4 are missing, or PipeWire was not
  restarted after copying them. `journalctl --user -u pipewire` says why a node
  failed to open.

mic_frontend or pw_audio_play: "cannot connect to the PipeWire daemon"
: PipeWire is not running (`loginctl show-user pollen | grep Linger` must say
  yes), or the container was started without `/run/user/1000` (reinstall
  `docker/reachy-mini-ros.service` from this repository).

`docker exec reachy-mini-ros ros2 ...` says "executable file not found in $PATH"
: `docker exec` runs the command without a shell and skips the entrypoint, so
  ROS is not sourced. Go through bash, which the image sets up to source ROS
  even when non-interactive: `docker exec reachy-mini-ros bash -c "ros2 ..."`.

## Packages

| package | what |
|---|---|
| `reachy_mini` | Metapackage: depends on all of the below and on the audio nodes (`mic_frontend`, `pw_audio_play`); ships `driver.launch.py`, the parameter files and a systemd unit for native installations. Install this one. |
| `reachy_mini_driver_cpp` | The servo driver in C++ (uses `reachy_mini_c`, Eigen, nlohmann_json): the one `driver.launch.py` starts by default. |
| `reachy_mini_driver` | The same driver in Python (uses Pollen's Python wheels), a reference implementation limited to `replay_motion` and `list_motions`. |
| `reachy_mini_c` | C library (Rust inside, built by cargo) over Pollen's motor-controller and kinematics crates. See its `VENDORING.md`. |
| `reachy_mini_msgs` | Interfaces: `msg/AntennasTarget`. |
| `reachy_mini_camera_driver` | The camera (IMX708 wide, CSI) through libcamera, H.264 from the Pi's hardware encoder. |
| `reachy_mini_description` | URDF and meshes (from the Pollen SDK model), restructured for `robot_state_publisher`, and the RViz configuration (model, TF, camera). |
| `docker/` | The robot's ROS 2 container image, its systemd units, the PipeWire configuration and `provision.sh` (not a ROS package). |
