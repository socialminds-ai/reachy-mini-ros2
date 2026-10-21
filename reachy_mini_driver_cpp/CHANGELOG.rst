^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package reachy_mini_driver_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.5.0 (2026-10-21)
------------------

1.4.0 (2026-10-13)
------------------
* driver: wake_up and go_to_sleep skills
  Wake-up and sleep only happened at startup and exit. They are now also
  actions, with standard interfaces:
  - /skill/wake_up (motions_skills/ReplayMotion): torque on, then a goto to
  the rest pose; with a name (e.g. wake-mini-up), plays that motion first,
  then the goto, so it always ends in the rest pose;
  - /skill/go_to_sleep (std_skills/Empty): a goto to the sleep pose; the
  motors stay on.
  Both run in the motion slot (the goto turned into a move), so priority,
  preemption, cancel and feedback work as for any motion, and they clear
  the look_at gaze. Implemented in the C++ and Python drivers, with tests;
  the C++ driver test gets a 180 s timeout (it went past ctest's 60 s).
  The goal-finishing code is shared (finishGoal / _finish).
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

1.3.1 (2026-10-08)
------------------
* driver: centre the body yaw on wake-up and sleep
  The driver adopted the body yaw measured at startup and the wake-up goto
  kept it, so a body left turned stayed turned. Wake-up and sleep now go to
  body yaw 0; look_at and reset leave it alone. The Python driver's gotos
  now interpolate the body yaw too.
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* driver: no crash on Ctrl+C while a goal runs
  On Ctrl+C the context is shut down and main() sends the robot to sleep
  while the goal threads still run: their next publish_feedback, or their
  final succeed/abort/canceled, threw RCLError in a detached thread, and
  std::terminate aborted the driver (exit -6).
  Goal threads are now started through runGoal(), which counts them and
  catches their exceptions (silently once the context is shut down: no
  client is left to report to). Feedback is not published after the
  shutdown, and shutdown() and the destructor wait (bounded) for the goal
  threads to end, so none fights the sleep goto or outlives the node.
  A test reproduces the Ctrl+C with a context of its own.
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

1.3.0 (2026-10-07)
------------------
* look_at: resolve the target once, and add the TRACKING policy
  A target expressed in a frame that moves with the gaze (head, camera, or
  the body, turned by the automatic body yaw of the IK) was re-transformed
  into base_link at every tick: it ran away from the head as it turned,
  and the head chased it. With an empty policy and GLANCE, the target is
  now transformed once, when the goal is received, and stays fixed; an
  unresolvable frame aborts the goal with ROS_EBADMSG.
  Following a moving frame is now the TRACKING policy (interaction_skills
  >= 1.9.1): the target is re-evaluated every tick, and the goal runs
  until cancelled, replaced by another gaze command (ROS_EINTR), or lost
  for tracking_lost_timeout seconds (ROS_EOTHER). Tracking one of the
  robot's own frames (URDF links from /robot_description, head, rods)
  other than base_frame cannot settle and logs a loud warning.
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

1.2.2 (2026-10-07)
------------------

1.2.1 (2026-10-05)
------------------
* Drivers: clear error, with the serial_port:=fake hint, when the servo bus cannot be opened
  Instead of 'terminate called after throwing...' (C++) or a traceback (Python),
  startup errors are logged as FATAL and the node exits with code 1.
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

1.2.0 (2026-10-05)
------------------
* Drivers: move libraries from installed packages (package://), by default
  MoveLibrary (C++ and Python) takes package://<package>/<directory>,
  resolved through the ament index. The default library is now
  package://reachy_mini_expressions_data/emotions, and the dances are an
  extra library: no HuggingFace cache or network needed, and the moves are
  versioned with the stack. Directories and HuggingFace dataset names still
  work.
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

1.1.0 (2026-10-05)
------------------

1.0.0 (2026-10-02)
------------------

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
