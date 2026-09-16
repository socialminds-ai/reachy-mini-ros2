^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package reachy_mini_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-09-16)
------------------
* Add the reachy_mini metapackage (launch file, parameters, systemd unit; C++ driver by default); ament_auto CMakeLists; maintainer email; self-contained README
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
* Import the Reachy Mini driver packages from the from-zero-to-ros tutorial
  reachy_mini_c (C ABI over Pollen's Rust crates), reachy_mini_msgs (PlayMove
  action), reachy_mini_driver (Python) and reachy_mini_driver_cpp (C++), all
  tested with colcon in a ros:lyrical container and on a real Reachy Mini.
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
* Contributors: Séverin Lemaignan
