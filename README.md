# Reachy Mini driver (ROS 2)

Bare-metal ROS 2 support for the Pollen Robotics Reachy Mini: nodes that own the
servo bus directly, without the Pollen daemon. Written for ROS 2 Lyrical.

| package | what |
|---|---|
| `reachy_mini_c` | C library (Rust inside, built by cargo) over Pollen's motor-controller and kinematics crates: `rmc_motors_*`, `rmc_kinematics_*`. See its `VENDORING.md`. |
| `reachy_mini_msgs` | Interfaces: `action/PlayMove`, `msg/AntennasTarget`. |
| `reachy_mini_driver` | The driver node in Python (Pollen's Python wheels for motors and kinematics). |
| `reachy_mini_driver_cpp` | The same driver in C++ (uses `reachy_mini_c`, Eigen, nlohmann_json). |

Both drivers: node `reachy_mini_driver`, executable `driver_node`, parameters
`serial_port` (`/dev/ttyAMA3` on the robot, or `fake` for an ideal simulated bus),
`rate` (50 Hz), `dataset` (move library), `wake_up_on_start`, `sleep_on_shutdown`.
They publish `joint_states` and `~/head_pose`, subscribe `~/target_head_pose`,
`~/target_antennas`, `~/target_body_yaw`, and offer the `~/play_move` action that
plays recorded moves of the Pollen emotions library.

```sh
rosdep install --from-paths . --ignore-src -y     # cargo, eigen, nlohmann-json, ...
colcon build && colcon test && colcon test-result --verbose
ros2 run reachy_mini_driver driver_node --ros-args -p serial_port:=fake
ros2 action send_goal /reachy_mini_driver/play_move reachy_mini_msgs/action/PlayMove "{move: cheerful1}"
```

The Python driver needs the `reachy-mini` pip package installed next to `rclpy`
(see `tools/` and the tutorial at https://gitlab.iiia.csic.es/socialminds/training/from-zero-to-ros,
Part 7). On the robot, stop the Pollen daemon first: only one process can own the serial port.

`tools/check_reachy_mini_c_against_sdk.py` compares the C library's kinematics with
the Python SDK (they agree to 1e-13 rad).
