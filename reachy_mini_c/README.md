# reachy_mini_c

A small C library (implemented in Rust) exposing what a C/C++ ROS 2 node needs
to drive a Reachy Mini **without the Pollen daemon**:

* `rmc_motors_*`: open the servo bus, read positions, write goal positions, torque on/off;
* `rmc_kinematics_*`: analytical inverse kinematics and iterative forward
  kinematics of the head (Stewart platform + body yaw), same conventions as the Python SDK.

It reuses Pollen Robotics' own crates (see `VENDORING.md`), so the numbers are
bit-for-bit those of the SDK (`../tools/check_reachy_mini_c_against_sdk.py`).

```cpp
#include <reachy_mini_c/reachy_mini_c.h>

RmcKinematics *kin = rmc_kinematics_new();
double pose[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0.01, 0,0,0,1};   // 1 cm up
double joints[RMC_NUM_JOINTS];
if (rmc_kinematics_ik(kin, pose, 0.0, joints) != 0) puts(rmc_last_error());

RmcMotors *m = rmc_motors_open("/dev/ttyAMA3");
double goals[RMC_NUM_MOTORS] = {joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], joints[6], 0.0, 0.0};
rmc_motors_set_torque(m, 1);
rmc_motors_write_goal_positions(m, goals);
```

Build: a normal `ament_cmake` package; needs `cargo` (`rosdep install` gets it).
CMake target for downstream packages: `reachy_mini_c::reachy_mini_c`.
