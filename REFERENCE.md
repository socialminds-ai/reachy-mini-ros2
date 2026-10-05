# Reachy Mini ROS 2: reference

The interface of the nodes, the launch file, and the installations that are not
the main path of [README.md](README.md) (native, from source, the Python
driver). The reasons behind the design are in [DESIGN.md](DESIGN.md).

## The launch file

`ros2 launch reachy_mini driver.launch.py` starts the whole robot: the servo
driver, `robot_state_publisher` with the robot model, the camera and the audio.

| argument | default | |
|---|---|---|
| `lang` | `cpp` | `cpp` or `python`: which driver implementation |
| `serial_port` | `/dev/ttyAMA3` | the servo bus, or `fake` for an ideal simulated bus |
| `params_file` | `share/reachy_mini/config/driver.yaml` | the driver's parameters |
| `description` | `true` | `robot_state_publisher` with the `reachy_mini_description` model |
| `camera` | `true` | `reachy_mini_camera_driver` |
| `audio` | `true` | `mic_frontend` and `pw_audio_play` |
| `rviz` | `false` | RViz with `share/reachy_mini_description/rviz/reachy_mini.rviz` |

## The driver node

Both implementations expose the same interface. Node `reachy_mini_driver`,
executable `driver_node`:

| | |
|---|---|
| parameters | `serial_port` (`/dev/ttyAMA3` on the robot, or `fake` for an ideal simulated bus), `rate` (50 Hz), `dataset` (default move library), `extra_datasets` (more libraries searched by the motion skills), `wake_up_on_start`, `sleep_on_shutdown`, `publish_tf`, `base_frame`, `skills_namespace` (`/skill`), `motion_initial_goto` (0.5 s), `glance_duration` (2 s, C++ only), the speed limits `max_speed.*`, `max_acceleration.*` and `safety_max_speed.*` (C++ only, see below) |
| publishes | `joint_states` (9 joints: body_yaw, stewart_1..6, right_antenna, left_antenna), `~/head_pose`, and on `/tf` the frames `robot_state_publisher` cannot derive from joint angles: `base_link -> head` (forward kinematics of the Stewart platform) and `base_link -> rod_1..6` |
| subscribes | `~/target_head_pose` (geometry_msgs/PoseStamped), `~/target_antennas` (reachy_mini_msgs/AntennasTarget), `~/target_body_yaw` (std_msgs/Float64) |
| skills | see below: the driver implements the [ROS4HRI skills](https://ros4hri.github.io/skills/) that map onto the hardware, under `skills_namespace` (`/skill` by default) |

The control loop runs at 50 Hz: read the 9 motor positions, publish them and the
head pose (forward kinematics), evaluate the move or goto in progress, inverse
kinematics, write the 9 goal positions. On exit the driver goes to the sleep
pose and releases the motors (`sleep_on_shutdown`, default true), so the Pollen
daemon can take over again cleanly.

**Speed limits** (C++ driver). Between the targets and the inverse kinematics, the
commanded pose follows the target at a bounded speed: the head moves in a straight
line and turns about a fixed axis, the antennas and the body yaw each move on their
own. While no motion plays (`look_at`, the `~/target_*` topics), the smooth limits
apply, with an acceleration ramp, so a new target is reached in a smooth movement
rather than at the servos' top speed. While a motion plays, only the safety caps
apply, high enough for the emotions library (whose fastest moves, `dying1` for
instance, reach 7 to 10 rad/s) but not for the jumps of a few milliseconds some
recordings contain. Gotos (wake-up, sleep, `reset`, the glide to a motion's first
sample) last as long as the smooth limits require, and start and stop at rest.

| part | `max_speed.` | `max_acceleration.` | `safety_max_speed.` |
|---|---|---|---|
| `head_rotation` | 1.5 rad/s | 6 rad/s² | 10 rad/s |
| `head_translation` | 0.1 m/s | 0.5 m/s² | 0.5 m/s |
| `antennas` | 4 rad/s | 20 rad/s² | 15 rad/s |
| `body_yaw` | 1.5 rad/s | 6 rad/s² | 4 rad/s |

A joint trajectory starts from the current joint positions (not at its first
point), and its antenna and body-yaw joints are held to their safety caps.

### Skills (ROS4HRI)

The driver is the *skill server* for what the servos can do. Any ROS4HRI-compatible
behaviour node can use it without knowing anything Reachy-specific: the goals carry a
`std_skills/Meta` (`caller`, `priority`), the results a `std_skills/Result`
(`error_code` + `error_msg`, codes from `std_skills/msg/Result`), feedback a
`std_skills/Feedback` (`data_float` = progress 0..1, `data_str` = what is playing).

| skill | type | C++ | Python | what |
|---|---|---|---|---|
| `/skill/replay_motion` | action `motions_skills/ReplayMotion` | yes | yes | plays a recorded move by `name`, from `dataset` (by default `package://reachy_mini_expressions_data/emotions`, Pollen's emotions library as installed by `reachy_mini_expressions_data`; also a local directory of move JSON files, or a HuggingFace dataset such as `pollen-robotics/reachy-mini-emotions-library`) or one of `extra_datasets` (by default the packaged dances); unknown name -> `ROS_EBADMSG` with the available names in `error_msg` |
| `/skill/list_motions` | service `motions_skills/ListMotions` | yes | yes | the names `replay_motion` accepts |
| `/skill/execute_cartesian_trajectory` | action `motions_skills/ExecuteCartesianTrajectory` | yes | no | `moveit_msgs/CartesianTrajectory` for `tracked_frame` `head`, poses in `base_frame` (the head frame sits about 0.177 m above `base_link` at rest, see `/tf`): head poses over time, linearly interpolated, IK by the driver |
| `/skill/execute_joint_trajectory` | action `motions_skills/ExecuteJointTrajectory` | yes | no | `trajectory_msgs/JointTrajectory` on any of the 9 joints, bypassing the kinematics; `safe_mode` (default true) refuses the `stewart_*` joints, since driving them one by one can strain the platform |
| `/skill/look_at` | action `interaction_skills/LookAt` | yes | no | `policy` empty: keep the head pointed at `target` (any TF frame, re-evaluated every tick) until a new goal or a cancel; `glance` (the `GLANCE` constant): look for `glance_duration` then come back; `reset`: back to the rest pose. `random`, `social` and `auto` (which need perception) return `ROS_ENOTSUP` |

One motion at a time: a new motion goal preempts the running one (which ends with
`ROS_EINTR`) unless its `meta.priority` is lower, in which case it is rejected. A cancel
returns `ROS_ECANCELED` and leaves the head where it is. Cartesian motions start with a
glide to their first sample (head, antennas and body yaw), of `motion_initial_goto`
seconds or longer if the distance requires it under the speed limits, so the head never
jumps.

```sh
ros2 service call /skill/list_motions motions_skills/srv/ListMotions
ros2 action send_goal /skill/replay_motion motions_skills/action/ReplayMotion "{name: cheerful1}"
ros2 action send_goal /skill/look_at interaction_skills/action/LookAt \
    "{policy: glance, target: {header: {frame_id: base_link}, point: {x: 1.0, y: 0.5, z: 0.3}}}"
```

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

## Camera: `reachy_mini_camera_driver`

The camera has a single owner, and the Pollen daemon opens it at startup: the
daemon must be stopped before the camera driver starts.

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
| stream parameters (dynamic; a change restarts the camera, about a second) | `width`, `height` (1280x720; the sensor is 16:9 and the hardware encoder stops at 1920x1080), `framerate` (30), `field_of_view` (`wide`: the full sensor, about 102° horizontally; or `narrow`: its central crop, about 77°, at most 1536 pixels wide, less work for the ISP; each needs its own calibration), `bitrate` (2 Mbit/s: the compression level), `keyframe_interval` (60 frames), `h264_profile` (`high`, `main` or `baseline`) |
| image parameters (dynamic, applied live) | `autofocus_mode` (`manual`, `continuous` or `auto`), `lens_position` (dioptres = 1/distance in m, for `manual`; 0.667 = 1.5 m), `exposure_value` (EV) |
| other parameters | `lazy` (true), `frame_id` (`camera_optical`, the URDF's optical frame), `camera_info_url` (empty: the shipped calibration of the sensor mode in use, `config/calibration_wide.yaml` or `calibration_narrow.yaml`, scaled to the other 16:9 sizes; or the URL of your own `camera_calibration` result), `source` (`libcamera`, `v4l2:<device>` or `test`) and `encoder` (`v4l2h264enc`, or `x264enc` / `openh264enc` to develop on a PC) |

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

`rpicam-hello --list-cameras` on the robot (daemon stopped) says whether the
kernel sees the camera.

## Audio: `mic_frontend` and `pw_audio_play`

Audio goes through PipeWire, with two nodes that live in their own
repositories, both started by `driver.launch.py`:

| node | | |
|---|---|---|
| `mic_frontend` | publishes | `/audio/head/raw` (`audio_common_msgs/AudioData`, 16 kHz mono S16LE, 10 ms chunks), `/audio/head/info`, `/audio/head/voice_detected` (`std_msgs/Bool`) |
| `pw_audio_play` | plays | `/audio/out` (`audio_common_msgs/AudioData`, in the format given on `/audio/out/info`); `/pw_audio_play/stop`, `/pw_audio_play/is_playing` |

On the robot, `docker/pipewire/` creates two PipeWire nodes over the audio card,
`reachy_mini_mic` and `reachy_mini_speaker`, through the same ALSA sharing
devices as the daemon, so they work whether the daemon runs or not (README.md,
section 1.4).

The card is an XMOS XVF3800: 16 kHz stereo only, and both capture channels
carry the same processed stream (4-mic beamforming, noise suppression, echo
cancellation against whatever the card plays). mic_frontend therefore needs
neither `aec` nor, usually, `rnnoise`: it runs `highpass,vad` on one channel.
These settings are in `reachy_mini/config/mic_frontend/` and
`reachy_mini/config/pw_audio_play/`; the `reachy_mini` package registers them
with launch_pal, which merges them over the nodes' own defaults whenever their
launch files run. Override them with a file in `~/.pal/config/`.

## Other installations

The main path (README.md) runs ROS on the robot in the `reachy-mini-ros`
container, whose image already has everything below.

### Native, from the SocialMinds apt repository

On Ubuntu 26.04 with ROS 2 Lyrical:

```sh
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://socialminds.doc.iiia.csic.es/apt/public.key | gpg --dearmor | sudo tee /etc/apt/keyrings/socialminds.gpg > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/socialminds.gpg] https://socialminds.doc.iiia.csic.es/apt/ lyrical main" | sudo tee /etc/apt/sources.list.d/socialminds.list
sudo apt update
sudo apt install socialminds-ros-lyrical-reachy-mini     # metapackage: drivers, msgs, model, launch, systemd unit
```

### The Python driver

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

The robot's image does this (`docker/install_reachy_mini_sdk.sh`). Then
`ros2 launch reachy_mini driver.launch.py lang:=python`.

### From source

```sh
cd ~/ros2_ws/src && git clone https://gitlab.iiia.csic.es/socialminds/robots/reachymini/reachy-mini-driver.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y     # cargo, rustc, eigen, nlohmann-json, yaml-cpp, ...
colcon build
colcon test && colcon test-result --verbose
source install/setup.bash
```

### The driver as a service, native installation

For a robot where ROS is installed natively (not the container of README.md,
whose unit is `docker/reachy-mini-driver-docker.service`), the `reachy_mini`
metapackage ships `share/reachy_mini/systemd/reachy-mini-driver.service`, which
runs `ros2 launch reachy_mini driver.launch.py`. It declares
`Conflicts=reachy-mini-daemon.service`, so systemd never runs the driver and the
daemon at the same time, and it stops the driver with SIGINT so the sleep
sequence runs.

```sh
sudo cp $(ros2 pkg prefix reachy_mini)/share/reachy_mini/systemd/reachy-mini-driver.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl disable reachy-mini-daemon        # do not race with the daemon at boot
sudo systemctl enable --now reachy-mini-driver
journalctl -u reachy-mini-driver -f
```

The unit assumes ROS 2 Lyrical in `/opt/ros/lyrical` and the `pollen` user
(member of `dialout` for the serial port). Edit `ExecStart` to add
`lang:=python` for the Python driver or `params_file:=...` for other parameters.

## Development

`tools/check_reachy_mini_c_against_sdk.py` compares the C library's kinematics
with the Python SDK's (they agree to 1e-13 rad); it needs the SDK installed.
Both drivers have unit tests that run on the fake bus (`colcon test`); the C++
driver's tests are gtest, the Python driver's are pytest. The Python driver is a
reference implementation limited to `replay_motion` and `list_motions`; the C++ driver
(the one `reachy_mini` installs) has the full skill set.

`reachy_mini_description/README.md` explains how the URDF is generated from the
SDK's model; `reachy_mini_c/VENDORING.md`, how Pollen's Rust crates are vendored.
