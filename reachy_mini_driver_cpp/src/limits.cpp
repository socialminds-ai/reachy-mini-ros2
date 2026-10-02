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

#include "reachy_mini_driver_cpp/limits.hpp"

#include <algorithm>
#include <cmath>

namespace reachy_mini_driver
{

double Approach::step(double distance, double dt, const SpeedLimit & limit)
{
  if (distance <= 0.0 || dt <= 0.0) {
    speed_ = 0.0;
    return 0.0;
  }
  double v = limit.velocity;
  if (limit.acceleration > 0.0) {
    v = std::min(v, speed_ + limit.acceleration * dt);
    // the speed from which braking at `acceleration` stops exactly at the target
    v = std::min(v, std::sqrt(2.0 * limit.acceleration * distance));
  }
  const double s = std::min(distance, std::max(v, 0.0) * dt);
  speed_ = s / dt;
  return s;
}

void MotionLimiter::reset(const Command & command)
{
  command_ = command;
  rotation_.reset();
  translation_.reset();
  body_yaw_.reset();
  for (auto & a : antennas_) {
    a.reset();
  }
}

namespace
{
// move `value` toward `target` by what `approach` allows
double approachScalar(
  double value, double target, Approach & approach, double dt, const SpeedLimit & limit)
{
  const double error = target - value;
  const double s = approach.step(std::abs(error), dt, limit);
  return value + std::copysign(s, error);
}
}  // namespace

const Command & MotionLimiter::step(const Command & target, double dt, const Limits & limits)
{
  // translation: along the straight line to the target
  const Eigen::Vector3d error = target.head.translation() - command_.head.translation();
  const double distance = error.norm();
  const double s = translation_.step(distance, dt, limits.head_translation);
  if (distance > 0.0) {
    command_.head.translation() += error * (s / distance);
  }
  // rotation: about the axis of the remaining rotation
  const Eigen::AngleAxisd remaining(
    command_.head.rotation().transpose() * target.head.rotation());
  const double angle = remaining.angle();
  const double r = rotation_.step(angle, dt, limits.head_rotation);
  if (angle > 0.0) {
    command_.head.linear() = command_.head.rotation() *
      Eigen::AngleAxisd(r, remaining.axis()).toRotationMatrix();
  }
  for (std::size_t i = 0; i < 2; ++i) {
    command_.antennas[i] = approachScalar(
      command_.antennas[i], target.antennas[i], antennas_[i], dt, limits.antennas);
  }
  command_.body_yaw = approachScalar(
    command_.body_yaw, target.body_yaw, body_yaw_, dt, limits.body_yaw);
  return command_;
}

double ease(double a)
{
  a = std::clamp(a, 0.0, 1.0);
  return a * a * (3.0 - 2.0 * a);
}

double gotoDuration(
  const Command & from, const Command & to, const Limits & limits,
  double requested)
{
  const double rotation = Eigen::AngleAxisd(
    from.head.rotation().transpose() * to.head.rotation()).angle();
  const double translation = (to.head.translation() - from.head.translation()).norm();
  const double antennas = std::max(
    std::abs(to.antennas[0] - from.antennas[0]), std::abs(to.antennas[1] - from.antennas[1]));
  const double body_yaw = std::abs(to.body_yaw - from.body_yaw);
  constexpr double kPeak = 1.5;   // peak / average speed of ease()
  double duration = requested;
  duration = std::max(duration, kPeak * rotation / limits.head_rotation.velocity);
  duration = std::max(duration, kPeak * translation / limits.head_translation.velocity);
  duration = std::max(duration, kPeak * antennas / limits.antennas.velocity);
  duration = std::max(duration, kPeak * body_yaw / limits.body_yaw.velocity);
  return duration;
}

}  // namespace reachy_mini_driver
