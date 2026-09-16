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

#ifndef REACHY_MINI_DRIVER_CPP__KINEMATICS_HPP_
#define REACHY_MINI_DRIVER_CPP__KINEMATICS_HPP_

// Thin C++ wrapper around reachy_mini_c's kinematics (Pollen's analytical IK
// and iterative FK), with Eigen types.

#include <Eigen/Geometry>

#include <array>
#include <memory>
#include <optional>

namespace reachy_mini_driver
{

using Pose = Eigen::Isometry3d;
using Joints = std::array<double, 7>;   // [body_yaw, stewart_1..6]

class Kinematics
{
public:
  Kinematics();
  ~Kinematics();
  Kinematics(const Kinematics &) = delete;
  Kinematics & operator=(const Kinematics &) = delete;

  /// Inverse kinematics with automatic body yaw; nullopt if the pose is not reachable.
  std::optional<Joints> ik(const Pose & head_pose, double body_yaw);
  /// Forward kinematics (iterative, warm-started); `iterations` >= 1.
  Pose fk(const Joints & joints, int iterations = 3);
  /// Reset the FK warm start.
  void resetFk(const Pose & head_pose);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/// Linear interpolation between two poses (translation lerp, rotation slerp), t in [0, 1].
Pose interpolate(const Pose & a, const Pose & b, double t);

}  // namespace reachy_mini_driver

#endif  // REACHY_MINI_DRIVER_CPP__KINEMATICS_HPP_
