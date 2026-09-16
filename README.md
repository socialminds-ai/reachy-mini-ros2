# Reachy Mini driver (ROS 2)

Bare-metal ROS 2 support for the Pollen Robotics Reachy Mini: nodes that own the
servo bus directly, without the Pollen daemon. Targets ROS 2 Lyrical.

| package | what |
|---|---|
| `reachy_mini_c` | C library (Rust inside, built by cargo) over Pollen's motor-controller and kinematics crates: `rmc_motors_*`, `rmc_kinematics_*`. See its `VENDORING.md`. |
| `reachy_mini_msgs` | Interfaces: `action/PlayMove`, `msg/AntennasTarget`. |
| `reachy_mini_driver` | The driver node in Python (uses Pollen's Python wheels for motors and kinematics). |
| `reachy_mini_driver_cpp` | The same driver in C++ (uses `reachy_mini_c`, Eigen, nlohmann_json). |
| `reachy_mini_description` | URDF and meshes (from the Pollen SDK model), restructured so that `robot_state_publisher` publishes what it can and the driver broadcasts the rest. |
| `reachy_mini` | Metapackage: depends on all of the above and ships the launch file, the parameter file and the systemd unit. Install this one. |

## The driver node

Both implementations expose the same interface. Node `reachy_mini_driver`,
executable `driver_node`:

| | |
|---|---|
| parameters | `serial_port` (`/dev/ttyAMA3` on the robot, or `fake` for an ideal simulated bus), `rate` (50 Hz), `dataset` (default move library), `wake_up_on_start`, `sleep_on_shutdown`, `publish_tf`, `base_frame` |
| publishes | `joint_states` (9 joints: body_yaw, stewart_1..6, right_antenna, left_antenna), `~/head_pose`, and on `/tf` the frames `robot_state_publisher` cannot derive from joint angles: `base_link -> head` (forward kinematics of the Stewart platform) and `base_link -> rod_1..6` |
| subscribes | `~/target_head_pose` (geometry_msgs/PoseStamped), `~/target_antennas` (reachy_mini_msgs/AntennasTarget), `~/target_body_yaw` (std_msgs/Float64) |
| action | `~/play_move` (reachy_mini_msgs/PlayMove): plays a recorded move of a library (a HuggingFace dataset such as Pollen's `pollen-robotics/reachy-mini-emotions-library`, or a local directory of move JSON files) |

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
# on the robot (Pollen daemon stopped, see below)
ros2 launch reachy_mini driver.launch.py                    # C++ driver (default)
ros2 launch reachy_mini driver.launch.py lang:=python       # Python driver

ros2 topic echo /joint_states
ros2 action send_goal /reachy_mini_driver/play_move reachy_mini_msgs/action/PlayMove "{move: cheerful1}"
```

The launch file takes `lang` (`cpp`, the default, or `python`), `serial_port` and `params_file`;
the default parameter file is `reachy_mini/config/driver.yaml` (installed under
`share/reachy_mini/config/`).

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
sound, camera and the Pollen apps. The driver puts the robot to sleep and
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

## Design

`DESIGN.md` explains where the design comes from: what the Pollen daemon does every
20 ms, the conventions inherited from the SDK, and how they shaped the ROS driver,
its action, its TF frames and its tests.

## Development

`tools/check_reachy_mini_c_against_sdk.py` compares the C library's kinematics
with the Python SDK's (they agree to 1e-13 rad); it needs the SDK installed.
Both drivers have unit tests that run on the fake bus (`colcon test`); the C++
driver's tests are gtest, the Python driver's are pytest.
