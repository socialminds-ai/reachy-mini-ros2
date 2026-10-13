^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package reachy_mini_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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
* Description: flip the antenna joint axes
  The SDK model has the antenna axes reversed (its MuJoCo backend negates the
  antenna commands), so RViz showed the antennas turning the wrong way.
  joint_states keep the motors' convention.
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

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

0.5.0 (2026-09-23)
------------------
* reachy_mini_description: drop hidden meshes, white head and black eyes
  - convert_sdk_urdf.py drops the visuals hidden inside the shells (servo
  cases, horns, screws, connectors, antenna holders, camera board, fisheye
  lenses), checked by rendering the model from many viewpoints:
  161 -> 45 visuals, 41 -> 23 meshes (9.4 -> 5.7 MB)
  - opaque black lenses and glasses, white neck piece
  - RViz paints all the visuals of a link with the first visual's material,
  so each link now carries a single colour and the visuals of other
  colours hang on fixed child links <link>_color<n>
  - unused meshes are removed from meshes/
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

0.4.0 (2026-09-17)
------------------

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
* Contributors: Séverin Lemaignan
