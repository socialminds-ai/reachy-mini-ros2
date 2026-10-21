^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package reachy_mini_c
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.5.0 (2026-10-21)
------------------

1.4.0 (2026-10-13)
------------------

1.3.1 (2026-10-08)
------------------

1.3.0 (2026-10-07)
------------------

1.2.2 (2026-10-07)
------------------

1.2.1 (2026-10-05)
------------------

1.2.0 (2026-10-05)
------------------

1.1.0 (2026-10-05)
------------------

1.0.0 (2026-10-02)
------------------

0.8.0 (2026-10-02)
------------------
* update changelogs
* Contributors: Séverin Lemaignan

0.7.0 (2026-10-01)
------------------

0.6.0 (2026-09-23)
------------------

0.5.0 (2026-09-23)
------------------

0.4.0 (2026-09-17)
------------------

0.3.0 (2026-09-16)
------------------
* DESIGN.md (background: the daemon's loop and how it shaped the driver); colcon runs the Python tests (extras_require test); publish_tf/base_frame in driver.yaml; lint and comment fixes
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

0.2.0 (2026-09-16)
------------------
* TF and robot model: reachy_mini_description (URDF + meshes from the SDK model, restructured for robot_state_publisher) and TF broadcasting of the head and rods from the drivers' kinematics
  - reachy_mini_description: converter script from the SDK URDF, 41 meshes,
  description.launch.py (robot_state_publisher, optional RViz), RViz config.
  The head and the six rods hang under base_link through continuous joints
  that never appear in joint_states, so robot_state_publisher leaves them to
  the driver.
  - reachy_mini_c: rmc_kinematics_lengths and rmc_kinematics_leg expose the
  platform geometry.
  - both drivers: base_link -> head (FK) and base_link -> rod_1..6 on /tf
  (publish_tf, base_frame parameters), with rod-length consistency tests.
  - reachy_mini/driver.launch.py starts robot_state_publisher (description:=,
  rviz:= arguments).
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
* Add the reachy_mini metapackage (launch file, parameters, systemd unit; C++ driver by default); ament_auto CMakeLists; maintainer email; self-contained README
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
* Import the Reachy Mini driver packages from the from-zero-to-ros tutorial
  reachy_mini_c (C ABI over Pollen's Rust crates), reachy_mini_msgs (PlayMove
  action), reachy_mini_driver (Python) and reachy_mini_driver_cpp (C++), all
  tested with colcon in a ros:lyrical container and on a real Reachy Mini.
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
* Contributors: Séverin Lemaignan
