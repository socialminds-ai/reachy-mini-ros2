^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package reachy_mini
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.1 (2026-10-08)
------------------
* reachy_mini: drop the rviz launch argument
  A driver launch file should not start RViz; run it separately (README
  section 2.3).
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
* reachy_mini: depend on reachy_mini_expressions_data
  Its driver.yaml uses the packaged move libraries by default.
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
* move the full RViz configuration to reachy_mini_description
  - reachy_mini_description/rviz/reachy_mini.rviz is now the full
  configuration (model, TF, camera image), replacing the model-only one;
  reachy_mini no longer installs an rviz/ directory
  - driver.launch.py rviz:=true opens it from reachy_mini_description
  - a developer machine only needs the description package to see the robot,
  its TF and its camera in RViz, not the drivers
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

0.6.0 (2026-09-23)
------------------
* reachy_mini: driver.launch.py starts the whole robot (servos, camera, audio)
  - driver.launch.py also starts the microphones (mic_frontend) and the
  speaker (pw_audio_play) through their own launch files; camera and audio
  are now on by default (camera:=false, audio:=false to leave them out)
  - config/mic_frontend/ and config/pw_audio_play/: the Reachy Mini settings
  of the audio nodes (reachy_mini_mic, one `head` channel, highpass,vad;
  target reachy_mini_speaker), registered as pal_configuration resources so
  that launch_pal merges them over the nodes' defaults
  - depends on mic_frontend and pw_audio_play
  - README and ROBOT_SETUP.md describe the full bring-up and the audio topics
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

0.5.0 (2026-09-23)
------------------
* reachy_mini: camera:=true, RViz config with the model, TF and the camera
  - depends on reachy_mini_camera_driver; driver.launch.py camera:=true
  includes its launch file (namespace /camera)
  - rviz/reachy_mini.rviz: grid, robot model, TF and the camera image
  (decoded from the H.264 stream by the ffmpeg image transport);
  driver.launch.py rviz:=true opens it instead of the description's
  model-only configuration
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

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
* Contributors: Séverin Lemaignan
