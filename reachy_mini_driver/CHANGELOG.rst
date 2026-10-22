^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package reachy_mini_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* driver: start asleep, publish the awake state on the robot blackboard; head taps
  The robot now starts asleep, and whether it is awake is robot-wide state
  on the robot blackboard (robot_blackboard, /robot_blackboard/awake), that
  other nodes follow:
  - both drivers: by default (wake_up_on_start false) a goto to the sleep
  pose at start, then the antennas wiggle twice (~1 s): the driver is
  ready. The awake state goes to `awake_parameter` (default
  /robot_blackboard/awake): false at start, set as soon as wake_up (true)
  or go_to_sleep (false) is called, before their motion, false at exit
  (C++: from a pre-shutdown callback, after a Ctrl+C). Written with an
  asynchronous parameter client; a 1 s timer writes it again when the
  blackboard appears or restarts, and retries a failed write. Tests: the
  asleep start with the wiggle, the blackboard following the skills (the
  Ctrl+C test now starts awake, as it assumes);
  - reachy_mini_head_taps (new package, moved from reachy_mini_tools so that
  the robot does not pull in face detection): 3 taps on the head call
  /skill/wake_up (Pollen's wake-mini-up the first time the robot wakes up,
  `rewake_motion`, none by default, afterwards), 4 taps /skill/go_to_sleep;
  taps for the state the robot is already in do nothing;
  - reachy_mini: driver.launch.py starts robot_blackboard (config
  robot_blackboard/10-reachy-mini.yml: awake false) and the head taps
  (`head_taps` argument, needs audio); asr_whistle follows the awake state
  (active_parameter) and only listens while the robot is awake.
  Docs: REFERENCE.md, DESIGN.md, README.md.
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

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
* Contributors: Séverin Lemaignan

1.3.0 (2026-10-07)
------------------

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
