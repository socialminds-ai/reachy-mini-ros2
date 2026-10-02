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


#include <gtest/gtest.h>

#include <cmath>

#include "reachy_mini_driver_cpp/limits.hpp"

using reachy_mini_driver::Approach;
using reachy_mini_driver::Command;
using reachy_mini_driver::Limits;
using reachy_mini_driver::MotionLimiter;
using reachy_mini_driver::Pose;
using reachy_mini_driver::SpeedLimit;

namespace
{
constexpr double kDt = 0.02;   // the driver's loop at 50 Hz
const Limits kLimits{{1.5, 6.0}, {0.1, 0.5}, {4.0, 20.0}, {1.5, 6.0}};

Command yawed(double yaw)
{
  Command c;
  c.head.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return c;
}

double yawOf(const Command & c)
{
  return Eigen::AngleAxisd(c.head.rotation()).angle();
}
}  // namespace

TEST(Approach, RespectsSpeedAndAccelerationAndStopsAtTheTarget)
{
  Approach a;
  const SpeedLimit limit{2.0, 10.0};
  double remaining = 3.0, speed = 0.0;
  int ticks = 0;
  while (remaining > 0.0 && ticks < 1000) {
    const double s = a.step(remaining, kDt, limit);
    const double v = s / kDt;
    EXPECT_LE(v, limit.velocity + 1e-9);
    EXPECT_LE(v - speed, limit.acceleration * kDt + 1e-9);
    remaining -= s;
    speed = v;
    ++ticks;
  }
  EXPECT_DOUBLE_EQ(remaining, 0.0);
  // 3 rad at 2 rad/s, plus 0.2 s to accelerate and as much to brake
  EXPECT_NEAR(ticks * kDt, 3.0 / 2.0 + 2.0 / 10.0, 0.1);
}

TEST(Approach, WithoutAccelerationLimitMovesAtFullSpeedAtOnce)
{
  Approach a;
  EXPECT_DOUBLE_EQ(a.step(1.0, kDt, SpeedLimit{5.0}), 5.0 * kDt);
  EXPECT_DOUBLE_EQ(a.step(0.01, kDt, SpeedLimit{5.0}), 0.01);
}

TEST(MotionLimiter, TurnsTheHeadSmoothlyInsteadOfJumping)
{
  MotionLimiter limiter;
  limiter.reset(yawed(0.0));
  const Command target = yawed(M_PI / 2);
  double previous = 0.0;
  int ticks = 0;
  while (std::fabs(yawOf(limiter.command()) - M_PI / 2) > 1e-6 && ticks < 500) {
    const double yaw = yawOf(limiter.step(target, kDt, kLimits));
    EXPECT_GE(yaw, previous - 1e-9);                                  // no overshoot back
    EXPECT_LE(yaw - previous, kLimits.head_rotation.velocity * kDt + 1e-9);
    previous = yaw;
    ++ticks;
  }
  EXPECT_LT(ticks, 500);
  // 90 degrees: about 1 s at 1.5 rad/s, plus the acceleration ramps
  EXPECT_GT(ticks * kDt, M_PI / 2 / kLimits.head_rotation.velocity);
  EXPECT_LT(ticks * kDt, 1.6);
}

TEST(MotionLimiter, FollowsAMovingTarget)
{
  MotionLimiter limiter;
  limiter.reset(Command{});
  Command target;
  for (int i = 0; i < 200; ++i) {
    target.head.translation().x() = 0.02 * std::sin(i * kDt);   // slow: within the limits
    target.antennas = {std::sin(i * kDt), -std::sin(i * kDt)};
    limiter.step(target, kDt, kLimits);
  }
  EXPECT_NEAR(limiter.command().head.translation().x(), target.head.translation().x(), 2e-3);
  EXPECT_NEAR(limiter.command().antennas[0], target.antennas[0], 0.05);
}

TEST(Goto, DurationGrowsWithTheDistance)
{
  const Command from = yawed(0.0);
  // a small move keeps the requested duration, a large one takes longer
  EXPECT_DOUBLE_EQ(gotoDuration(from, yawed(0.1), kLimits, 0.5), 0.5);
  EXPECT_NEAR(gotoDuration(from, yawed(1.5), kLimits, 0.5), 1.5 * 1.5 / 1.5, 1e-9);
  Command antennas = from;
  antennas.antennas = {-3.0, 3.0};
  EXPECT_NEAR(gotoDuration(from, antennas, kLimits, 0.5), 1.5 * 3.0 / 4.0, 1e-9);
  // the easing starts and ends at rest
  EXPECT_DOUBLE_EQ(reachy_mini_driver::ease(0.0), 0.0);
  EXPECT_DOUBLE_EQ(reachy_mini_driver::ease(1.0), 1.0);
  EXPECT_NEAR(reachy_mini_driver::ease(1e-3), 0.0, 1e-5);
}
