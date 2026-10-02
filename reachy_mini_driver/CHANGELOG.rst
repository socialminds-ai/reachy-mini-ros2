^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package reachy_mini_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-10-02)
------------------

0.8.0 (2026-10-02)
------------------
* update changelogs
* Contributors: Séverin Lemaignan

0.7.0 (2026-10-01)
------------------
* docs: README follows the main path (installation, usage, tests)
  - README.md: installation (an off-the-shelf Reachy Mini to a ROS robot,
  with the Pollen daemon or ROS as the default at boot), usage (start ROS
  on the robot, connect a developer machine or container over zenoh, RViz),
  quick tests, troubleshooting
  - ROBOT_SETUP.md merged into the README and removed
  - REFERENCE.md (new): launch arguments, driver interface and skills, TF,
  camera and audio details, native and from-source installations
  - references to ROBOT_SETUP.md updated in docker/, DESIGN.md and
  reachy_mini_driver/package.xml
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

0.6.0 (2026-09-23)
------------------

0.5.0 (2026-09-23)
------------------

0.4.0 (2026-09-17)
------------------
* Adopt the ROS4HRI skill API instead of the private PlayMove action
  The drivers now serve the standard skills from https://ros4hri.github.io/skills.json
  under the `skills_namespace` parameter (/skill by default), with std_skills
  Meta/Result/Feedback semantics (priority preemption -> ROS_EINTR, lower priority
  rejected, cancel -> ROS_ECANCELED, unknown motion -> ROS_EBADMSG with the list of
  available names).
  C++ driver (reachy_mini_driver_cpp), full set:
  - replay_motion (action) and list_motions (service), over `dataset` + `extra_datasets`
  - execute_cartesian_trajectory (moveit_msgs/CartesianTrajectory, head poses in base_frame)
  - execute_joint_trajectory (trajectory_msgs/JointTrajectory, safe_mode protects stewart\_*)
  - look_at: target tracking through tf2, glance (glance_duration) and reset; the policies
  needing perception answer ROS_ENOTSUP
  - one Motion interface (trajectories.hpp) behind a single motion slot
  Python driver: replay_motion + list_motions only (reference implementation).
  reachy_mini_msgs keeps only AntennasTarget; PlayMove.action is removed. New
  parameters skills_namespace, extra_datasets, motion_initial_goto, glance_duration.
  Tests run in private ROS domains so colcon can run the packages in parallel.
  Docs (README, DESIGN, ROBOT_SETUP, driver.yaml) updated; version 0.3.0.
  Verified on the fake bus (gtest/pytest) and on a Reachy Mini from a laptop over zenoh.
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

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
