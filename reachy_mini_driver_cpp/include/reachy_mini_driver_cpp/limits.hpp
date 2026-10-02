// Copyright 2026 SocialMinds Lab, IIIA-CSIC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef REACHY_MINI_DRIVER_CPP__LIMITS_HPP_
#define REACHY_MINI_DRIVER_CPP__LIMITS_HPP_

#include <array>

#include "reachy_mini_driver_cpp/kinematics.hpp"

namespace reachy_mini_driver
{

/// Speed (units/s) and acceleration (units/s²) limits; acceleration <= 0: unlimited.
struct SpeedLimit
{
  double velocity;
  double acceleration{0.0};
};

/// Limits for each part of the robot: head rotation (rad), head translation (m),
/// antennas (rad, each), body yaw (rad).
struct Limits
{
  SpeedLimit head_rotation, head_translation, antennas, body_yaw;
};

/// What the loop sends to the kinematics: head pose, antennas, body yaw.
struct Command
{
  Pose head{Pose::Identity()};
  std::array<double, 2> antennas{};
  double body_yaw{0.0};
};

/// One quantity closing the distance to its target: accelerates up to the speed
/// limit, and slows down so as to stop at the target (bang-bang on the speed).
class Approach
{
public:
  /// Distance (>= 0, at most `distance`) to cover during this tick.
  double step(double distance, double dt, const SpeedLimit & limit);
  void reset() {speed_ = 0.0;}

private:
  double speed_{0.0};
};

/// Follows a target command at bounded speed and acceleration: the head moves in
/// a straight line and rotates about a fixed axis toward the target, antennas
/// and body yaw move independently. The target may change at every tick.
class MotionLimiter
{
public:
  /// Jump to `command` (e.g. where the robot is), at rest.
  void reset(const Command & command);
  /// Advance by `dt` seconds toward `target`; returns the new command.
  const Command & step(const Command & target, double dt, const Limits & limits);
  const Command & command() const {return command_;}

private:
  Command command_;
  Approach rotation_, translation_, body_yaw_;
  std::array<Approach, 2> antennas_;
};

/// Smooth start and stop for a goto: s(0) = 0, s(1) = 1, zero speed at both
/// ends; peak speed 1.5 times the average.
double ease(double a);

/// The duration a goto from `from` to `to` needs so that its peak speed (eased)
/// stays within `limits`; at least `requested`.
double gotoDuration(
  const Command & from, const Command & to, const Limits & limits,
  double requested);

}  // namespace reachy_mini_driver

#endif  // REACHY_MINI_DRIVER_CPP__LIMITS_HPP_
