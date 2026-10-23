^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package reachy_mini
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.7.0 (2026-10-23)
------------------
* reachy_mini: speech synthesis with tts_pockettts
  driver.launch.py starts tts_pockettts (tts:=false to skip it; needs the
  audio), configured by config/tts_pockettts/: the small English model,
  in the alba voice. The metapackage depends on tts_pockettts, its small
  English model and tts_pockettts_custom_voices, whose voices can then be
  selected with the voice parameter.
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

1.6.0 (2026-10-22)
------------------
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
* reachy_mini: one-line summary in the driver launch docstring (pep257)
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

1.5.0 (2026-10-21)
------------------
* reachy_mini: speech recognition with asr_whistle
  driver.launch.py starts asr_whistle on the head microphone (new `asr`
  argument, default true; needs `audio`), and reachy_mini depends on it.
  config/asr_whistle/10-reachy-mini.yml, registered with launch_pal,
  holds the settings measured on the robot: 3 inference threads (faster
  than 4 on the CM4, where a 4th thread waits on cores busy with the
  driver and the audio, and one core left free), a 500 ms VAD hangover
  (the head mic's VAD drops out between words), a 25 dB loudness-range
  gate (speech 32-40 dB, the robot's and room's noise mostly 9-24 dB),
  and robot_speaking remapped to /pw_audio_play/is_playing.
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

1.4.0 (2026-10-13)
------------------

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
