# Reachy Mini driver (ROS 2)

Bare-metal ROS 2 support for the Pollen Robotics Reachy Mini: nodes that own the
servo bus directly, without the Pollen daemon. Targets ROS 2 Lyrical.

| package | what |
|---|---|
| `reachy_mini_c` | C library (Rust inside, built by cargo) over Pollen's motor-controller and kinematics crates: `rmc_motors_*`, `rmc_kinematics_*`. See its `VENDORING.md`. |
| `reachy_mini_msgs` | Interfaces: `msg/AntennasTarget` (the skills use the standard ROS4HRI packages `motions_skills`, `interaction_skills`, `std_skills`). |
| `reachy_mini_driver` | The driver node in Python (uses Pollen's Python wheels for motors and kinematics). |
| `reachy_mini_driver_cpp` | The same driver in C++ (uses `reachy_mini_c`, Eigen, nlohmann_json). |
| `reachy_mini_camera_driver` | The camera (IMX708 wide, CSI) through libcamera: H.264 from the Pi's hardware encoder for `ffmpeg_image_transport` and Foxglove, raw frames on demand. See "Camera and audio" below. |
| `reachy_mini_description` | URDF and meshes (from the Pollen SDK model), restructured so that `robot_state_publisher` publishes what it can and the driver broadcasts the rest. |
| `reachy_mini` | Metapackage: depends on all of the above and on the audio nodes (`mic_frontend`, `pw_audio_play`), and ships the launch file that starts them all, the parameter files, the RViz configuration and the systemd unit. Install this one. |
| `docker/` | The ROS 2 container image for the robot and its systemd units (not a ROS package); see `ROBOT_SETUP.md`. |

## The driver node

Both implementations expose the same interface. Node `reachy_mini_driver`,
executable `driver_node`:

| | |
|---|---|
| parameters | `serial_port` (`/dev/ttyAMA3` on the robot, or `fake` for an ideal simulated bus), `rate` (50 Hz), `dataset` (default move library), `extra_datasets` (more libraries searched by the motion skills), `wake_up_on_start`, `sleep_on_shutdown`, `publish_tf`, `base_frame`, `skills_namespace` (`/skill`), `motion_initial_goto` (0.5 s), `glance_duration` (2 s, C++ only) |
| publishes | `joint_states` (9 joints: body_yaw, stewart_1..6, right_antenna, left_antenna), `~/head_pose`, and on `/tf` the frames `robot_state_publisher` cannot derive from joint angles: `base_link -> head` (forward kinematics of the Stewart platform) and `base_link -> rod_1..6` |
| subscribes | `~/target_head_pose` (geometry_msgs/PoseStamped), `~/target_antennas` (reachy_mini_msgs/AntennasTarget), `~/target_body_yaw` (std_msgs/Float64) |
| skills | see below: the driver implements the [ROS4HRI skills](https://ros4hri.github.io/skills.json) that map onto the hardware, under `skills_namespace` (`/skill` by default) |

### Skills (ROS4HRI)

The driver is the *skill server* for what the servos can do. Any ROS4HRI-compatible
behaviour node can use it without knowing anything Reachy-specific: the goals carry a
`std_skills/Meta` (`caller`, `priority`), the results a `std_skills/Result`
(`error_code` + `error_msg`, codes from `std_skills/msg/Result`), feedback a
`std_skills/Feedback` (`data_float` = progress 0..1, `data_str` = what is playing).

| skill | type | C++ | Python | what |
|---|---|---|---|---|
| `/skill/replay_motion` | action `motions_skills/ReplayMotion` | yes | yes | plays a recorded move by `name`, from `dataset` (a HuggingFace dataset such as Pollen's `pollen-robotics/reachy-mini-emotions-library`, or a local directory of move JSON files) or one of `extra_datasets`; unknown name -> `ROS_EBADMSG` with the available names in `error_msg` |
| `/skill/list_motions` | service `motions_skills/ListMotions` | yes | yes | the names `replay_motion` accepts |
| `/skill/execute_cartesian_trajectory` | action `motions_skills/ExecuteCartesianTrajectory` | yes | no | `moveit_msgs/CartesianTrajectory` for `tracked_frame` `head`, poses in `base_frame` (the head frame sits about 0.177 m above `base_link` at rest, see `/tf`): head poses over time, linearly interpolated, IK by the driver |
| `/skill/execute_joint_trajectory` | action `motions_skills/ExecuteJointTrajectory` | yes | no | `trajectory_msgs/JointTrajectory` on any of the 9 joints, bypassing the kinematics; `safe_mode` (default true) refuses the `stewart_*` joints, since driving them one by one can strain the platform |
| `/skill/look_at` | action `interaction_skills/LookAt` | yes | no | `policy` empty: keep the head pointed at `target` (any TF frame, re-evaluated every tick) until a new goal or a cancel; `glance` (the `GLANCE` constant): look for `glance_duration` then come back; `reset`: back to the rest pose. `random`, `social` and `auto` (which need perception) return `ROS_ENOTSUP` |

One motion at a time: a new motion goal preempts the running one (which ends with
`ROS_EINTR`) unless its `meta.priority` is lower, in which case it is rejected. A cancel
returns `ROS_ECANCELED` and leaves the head where it is. Cartesian motions start with a
`motion_initial_goto` glide to their first sample so the head never jumps.

The control loop runs at 50 Hz: read the 9 motor positions, publish them and the
head pose (forward kinematics), evaluate the move or goto in progress, inverse
kinematics, write the 9 goal positions. On exit the driver goes to the sleep
pose and releases the motors.

### TF and the robot model

The Stewart platform is a closed kinematic chain whose passive joints are not
measured, so `robot_state_publisher` alone cannot place the head. The split is:

* `robot_state_publisher` (started by `driver.launch.py`, model from
  `reachy_mini_description`) publishes the fixed frames and the revolute joints
  it gets from `joint_states`: body yaw, the six motor horns, the two antennas,
  the camera frames (`camera`, `camera_optical`) attached to the head;
* the driver broadcasts `base_link -> head` from its forward kinematics and
  `base_link -> rod_1..6` from the horn tips and the platform anchors, at the
  loop rate (`publish_tf`, `base_frame` parameters).

`ros2 launch reachy_mini driver.launch.py serial_port:=fake rviz:=true` shows the
model moving in RViz without a robot.

## On a Reachy Mini: the ROS 2 container

The robot runs Debian, which has no ROS 2 Lyrical binaries, so ROS runs there in
the Docker image of `docker/` (built by the `robot-image` pipeline job). See
**[ROBOT_SETUP.md](ROBOT_SETUP.md)** for the step-by-step preparation of an
off-the-shelf robot: Docker, the image, the systemd units, running the driver
instead of the Pollen daemon.

## Installation

### From the SocialMinds apt repository (recommended on robots)

```sh
# once: trust the repository (Ubuntu 26.04 / ROS 2 Lyrical)
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://socialminds.doc.iiia.csic.es/apt/public.key | gpg --dearmor | sudo tee /etc/apt/keyrings/socialminds.gpg > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/socialminds.gpg] https://socialminds.doc.iiia.csic.es/apt/ lyrical main" | sudo tee /etc/apt/sources.list.d/socialminds.list
sudo apt update
sudo apt install socialminds-ros-lyrical-reachy-mini     # metapackage: both drivers, msgs, launch, systemd unit
```

The Python driver additionally needs the `reachy-mini` pip package (Pollen's SDK,
which contains the motor-controller and kinematics wheels and the move loader)
importable by the same interpreter as `rclpy`. On Ubuntu 26.04 (Python 3.14) the
SDK's pinned `pygobject` does not install; use the distro's `python3-gi` and
install the SDK without that dependency:

```sh
sudo apt install python3-pip python3-gi python3-gst-1.0 gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0
python3 -m pip install --break-system-packages --no-deps reachy-mini
python3 -m pip install --break-system-packages $(python3 -c "import importlib.metadata as m; from packaging.requirements import Requirement; print(' '.join(str(r.name)+str(r.specifier) for r in map(Requirement, m.requires('reachy-mini')) if (not r.marker or r.marker.evaluate({'extra': ''})) and r.name.lower() != 'pygobject'))")
```

### From source

```sh
cd ~/ros2_ws/src && git clone https://gitlab.iiia.csic.es/socialminds/robots/reachymini/reachy-mini-driver.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y     # cargo, rustc, eigen, nlohmann-json, yaml-cpp, ...
colcon build
colcon test && colcon test-result --verbose
source install/setup.bash
```

## Running

```sh
# without hardware: an ideal simulated bus
ros2 run reachy_mini_driver driver_node --ros-args -p serial_port:=fake
# on the robot (Pollen daemon stopped, see below): servos, model, camera, audio
ros2 launch reachy_mini driver.launch.py                    # C++ driver (default)
ros2 launch reachy_mini driver.launch.py lang:=python       # Python driver

ros2 topic echo /joint_states
ros2 service call /skill/list_motions motions_skills/srv/ListMotions
ros2 action send_goal /skill/replay_motion motions_skills/action/ReplayMotion "{name: cheerful1}"
ros2 action send_goal /skill/look_at interaction_skills/action/LookAt \
    "{policy: glance, target: {header: {frame_id: base_link}, point: {x: 1.0, y: 0.5, z: 0.3}}}"
```

`driver.launch.py` starts the whole robot: the servo driver, `robot_state_publisher`
with the robot model, the camera and the audio (see "Camera and audio"). It takes
`lang` (`cpp`, the default, or `python`), `serial_port` and `params_file` for the
driver (the default parameter file is `reachy_mini/config/driver.yaml`, installed
under `share/reachy_mini/config/`), and `camera`, `audio`, `description` and
`rviz` to turn the rest on or off (all on except `rviz`).

## On the robot: stop the Pollen daemon first

Only one process can own the servo bus (`/dev/ttyAMA3`). The Pollen daemon
(`reachy-mini-daemon`, started at boot) holds it, so the driver cannot open the
port while the daemon runs, and vice versa. On the robot (user `pollen`):

```sh
sudo systemctl stop reachy-mini-daemon       # until the next boot
sudo systemctl disable reachy-mini-daemon    # also at boot, if the driver becomes permanent
# ... run the driver ...
sudo systemctl start reachy-mini-daemon      # give the robot back to Pollen's stack
```

Stopping the daemon also stops what it provides: its HTTP API on port 8000,
WebRTC and the Pollen apps (camera and sound are available in ROS instead, see
"Camera and audio"). The driver puts the robot to sleep and
releases the motors when it exits (`sleep_on_shutdown`, default true), so the
daemon can take over again cleanly.

## Running the driver as a service

The `reachy_mini` metapackage ships a systemd unit,
`share/reachy_mini/systemd/reachy-mini-driver.service`, which runs
`ros2 launch reachy_mini driver.launch.py` (the C++ driver). It declares
`Conflicts=reachy-mini-daemon.service`, so systemd never runs the driver and the
daemon at the same time, and it stops the driver with SIGINT so the sleep
sequence runs.

```sh
sudo cp $(ros2 pkg prefix reachy_mini)/share/reachy_mini/systemd/reachy-mini-driver.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl disable reachy-mini-daemon        # do not race with the daemon at boot
sudo systemctl enable --now reachy-mini-driver
systemctl status reachy-mini-driver
journalctl -u reachy-mini-driver -f
```

The unit assumes a native ROS 2 Lyrical installation in `/opt/ros/lyrical` and
the `pollen` user (member of `dialout` for the serial port). Edit `ExecStart` to
add `lang:=python` for the Python driver or `params_file:=...` for other parameters, or,
if ROS runs in a container on the robot, to wrap the command with
`docker exec <container> ...`.

To go back to the Pollen stack: `sudo systemctl disable --now reachy-mini-driver
&& sudo systemctl enable --now reachy-mini-daemon`.

## Camera and audio

The Pollen daemon owns the camera and the audio card while it runs. Without it
(or next to it, for the audio), ROS gets them as follows; `ROBOT_SETUP.md`
(step 7) prepares the robot.

### Camera: `reachy_mini_camera_driver`

Node `camera`, executable `camera_node`, started by `driver.launch.py`, or on its
own by `ros2 launch reachy_mini_camera_driver camera.launch.py` (namespace `/camera`). A GStreamer pipeline
does all the work off the ROS thread: libcamera (the ISP scales and converts),
a tee, the Raspberry Pi's hardware H.264 encoder on one branch, raw frames on
the other. Only what has subscribers costs anything: each branch is shut by a
valve while nobody listens, and with `lazy` (the default) the camera itself
stops 5 s after the last subscriber leaves.

| | |
|---|---|
| publishes | `image_raw/ffmpeg` (`ffmpeg_image_transport_msgs/FFMPEGPacket`, H.264): subscribe with the `ffmpeg` image transport to get `sensor_msgs/Image`s, e.g. `ros2 run image_transport republish --ros-args -p in_transport:=ffmpeg -p out_transport:=raw -r in/ffmpeg:=/camera/image_raw/ffmpeg -r out:=/camera/image`; `image_raw/h264` (`foxglove_msgs/CompressedVideo`) for Foxglove; `image_raw` (`sensor_msgs/Image`, `yuv422_yuy2`, uncompressed: meant for nodes on the robot); `camera_info` |
| stream parameters (dynamic; a change restarts the camera, about a second) | `width`, `height` (1280x720; the sensor is 16:9 and the hardware encoder stops at 1920x1080), `framerate` (30), `bitrate` (2 Mbit/s: the compression level), `keyframe_interval` (60 frames), `h264_profile` (`high`, `main` or `baseline`) |
| image parameters (dynamic, applied live) | `autofocus_mode` (`continuous`, `auto`, `manual`), `lens_position` (dioptres, for `manual`), `exposure_value` (EV) |
| other parameters | `lazy` (true), `frame_id` (`camera_optical`, the URDF's optical frame), `camera_info_url` (a `camera_calibration` result; none is shipped, the daemon's calibration being for another camera), `source` (`libcamera`, `v4l2:<device>` or `test`) and `encoder` (`v4l2h264enc`, or `x264enc` / `openh264enc` to develop on a PC) |

Timestamps are the capture time (sensor timestamp from libcamera, in system
time). A new subscriber to the H.264 topics triggers a keyframe, so it does not
wait for the next one; SPS/PPS are repeated before every keyframe.

Each `ros2 param set` of a stream parameter restarts the camera. To change
several at once with a single restart, load them together:

```sh
cat > /tmp/1080p.yaml <<EOF
/camera/camera:
  ros__parameters: {width: 1920, height: 1080, bitrate: 4000000}
EOF
ros2 param load /camera/camera /tmp/1080p.yaml
```

Cost on the CM4 (one core = 100 %): 8 % at 720p30, 10 % at 1080p30. The
encoder reads libcamera's buffers in place, and most of that is libcamera's
own control loop (exposure, white balance, focus). A subscriber to the raw
`image_raw` adds about 30 % (copy and serialisation of the raw frames).

Without a robot: `ros2 run reachy_mini_camera_driver camera_node --ros-args -p source:=test -p encoder:=x264enc`.

To watch it from a laptop, with the robot model and TF:
`rviz2 -d $(ros2 pkg prefix reachy_mini)/share/reachy_mini/rviz/reachy_mini.rviz`
(the config `driver.launch.py rviz:=true` opens). The laptop needs
`ros-lyrical-ffmpeg-image-transport` for RViz to decode the H.264.

### Audio: `mic_frontend` and `pw_audio_play`

Audio goes through PipeWire, with two nodes that live in their own
repositories, both started by `driver.launch.py`:

| node | | |
|---|---|---|
| `mic_frontend` | publishes | `/audio/head/raw` (`audio_common_msgs/AudioData`, 16 kHz mono S16LE, 10 ms chunks), `/audio/head/info`, `/audio/head/voice_detected` (`std_msgs/Bool`) |
| `pw_audio_play` | plays | `/audio/out` (`audio_common_msgs/AudioData`, in the format given on `/audio/out/info`); `/pw_audio_play/stop`, `/pw_audio_play/is_playing` |

On the robot, `docker/pipewire/` creates two PipeWire nodes over the audio card,
`reachy_mini_mic` and `reachy_mini_speaker`, through the same ALSA sharing
devices as the daemon, so they work whether the daemon runs or not.

The card is an XMOS XVF3800: 16 kHz stereo only, and both capture channels
carry the same processed stream (4-mic beamforming, noise suppression, echo
cancellation against whatever the card plays). mic_frontend therefore needs
neither `aec` nor, usually, `rnnoise`: it runs `highpass,vad` on one channel.
These settings are in `reachy_mini/config/mic_frontend/` and
`reachy_mini/config/pw_audio_play/`; the `reachy_mini` package registers them
with launch_pal, which merges them over the nodes' own defaults whenever their
launch files run. Override them with a file in `~/.pal/config/`.

## Design

`DESIGN.md` explains where the design comes from: what the Pollen daemon does every
20 ms, the conventions inherited from the SDK, and how they shaped the ROS driver,
its skills, its TF frames and its tests.

## Development

`tools/check_reachy_mini_c_against_sdk.py` compares the C library's kinematics
with the Python SDK's (they agree to 1e-13 rad); it needs the SDK installed.
Both drivers have unit tests that run on the fake bus (`colcon test`); the C++
driver's tests are gtest, the Python driver's are pytest. The Python driver is a
reference implementation limited to `replay_motion` and `list_motions`; the C++ driver
(the one `reachy_mini` installs) has the full skill set.
