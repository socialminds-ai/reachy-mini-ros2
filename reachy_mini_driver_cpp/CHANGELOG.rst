^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package reachy_mini_driver_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.8.0 (2026-10-02)
------------------
* C++ driver: speed limits, smooth gotos, joint trajectories from the current position
  The commanded pose now follows the targets at bounded speed (MotionLimiter,
  limits.hpp): smooth limits with an acceleration ramp for look_at and the
  target topics, speed caps only while a motion plays (chosen from the
  emotions library: dying1 keeps its timing, the millisecond jumps of
  wake-mini-up are spread out). Gotos last as long as the distance needs,
  are eased, and bring the body yaw too. Joint trajectories start from the
  current goals instead of jumping to their first point.
  Also: a goal cancelled by the driver itself (shutdown) is aborted instead
  of 'canceled', which rclcpp refused, aborting the process before the robot
  went to sleep.
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* update changelogs
* docs: architecture diagram, image pruning, registry access
  - doc/architecture.svg replaces the ASCII diagram of the README: the ROS
  container and the Pollen daemon on the robot, the hardware they own, and
  the developer machine connected over zenoh
  - updating the image now prunes the previous one (README 1.6, provision.sh):
  each update otherwise leaves ~4 GB behind and fills the 14 GB card;
  rebuilding uses --no-cache so that apt fetches the new packages
  - the pre-built image is only available to IIIA staff for now; the README
  says so and gives the build-it-yourself path for installs and updates
  - provision.sh is presented as a tip, instead of the manual steps
  - the pollen account's default password is root
  - ROS4HRI skills link: https://ros4hri.github.io/skills/
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

* docs: architecture diagram, image pruning, registry access
  - doc/architecture.svg replaces the ASCII diagram of the README: the ROS
  container and the Pollen daemon on the robot, the hardware they own, and
  the developer machine connected over zenoh
  - updating the image now prunes the previous one (README 1.6, provision.sh):
  each update otherwise leaves ~4 GB behind and fills the 14 GB card;
  rebuilding uses --no-cache so that apt fetches the new packages
  - the pre-built image is only available to IIIA staff for now; the README
  says so and gives the build-it-yourself path for installs and updates
  - provision.sh is presented as a tip, instead of the manual steps
  - the pollen account's default password is root
  - ROS4HRI skills link: https://ros4hri.github.io/skills/
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

0.7.0 (2026-10-01)
------------------

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
