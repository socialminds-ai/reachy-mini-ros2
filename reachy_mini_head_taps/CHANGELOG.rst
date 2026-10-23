^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package reachy_mini_head_taps
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.7.0 (2026-10-23)
------------------

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
* Contributors: Séverin Lemaignan

1.5.0 (2026-10-21)
------------------

1.4.0 (2026-10-13)
------------------

1.3.1 (2026-10-08)
------------------

1.3.0 (2026-10-07 17:33)
------------------------

1.2.2 (2026-10-07 13:23)
------------------------

1.2.1 (2026-10-05 18:02)
------------------------

1.2.0 (2026-10-05 14:13)
------------------------

1.1.0 (2026-10-05 13:38)
------------------------

1.0.0 (2026-10-02 15:36)
------------------------

0.8.0 (2026-10-02 11:48)
------------------------

0.7.0 (2026-10-01)
------------------

0.6.0 (2026-09-23 17:15)
------------------------

0.5.0 (2026-09-23 16:34)
------------------------

0.4.0 (2026-09-17)
------------------

0.3.0 (2026-09-16 18:02)
------------------------

0.2.0 (2026-09-16 17:46)
------------------------
