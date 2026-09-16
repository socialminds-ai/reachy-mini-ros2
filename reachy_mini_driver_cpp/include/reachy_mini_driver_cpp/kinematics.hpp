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
#include <vector>

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

  /// The `head` frame in the base frame: the FK pose plus the rest height.
  Pose headInBase(const Pose & head_pose) const;
  /// Pose of each of the 6 rods in the base frame: origin at the horn tip, x axis
  /// towards the anchor on the platform (rods are symmetric, roll is arbitrary).
  std::vector<Pose> rodsInBase(const Joints & joints, const Pose & head_pose) const;
  /// Distance horn tip <-> anchor of each rod (equals rodLength() for a consistent state).
  std::vector<double> rodLengths(const Joints & joints, const Pose & head_pose) const;
  double rodLength() const {return rod_length_;}
  double headZOffset() const {return head_z_offset_;}

private:
  struct Leg
  {
    Pose t_base_motor;
    Eigen::Vector3d anchor_head;
  };
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::vector<Leg> legs_;
  double arm_length_{0.0};
  double rod_length_{0.0};
  double head_z_offset_{0.0};
};

/// Linear interpolation between two poses (translation lerp, rotation slerp), t in [0, 1].
Pose interpolate(const Pose & a, const Pose & b, double t);

}  // namespace reachy_mini_driver

#endif  // REACHY_MINI_DRIVER_CPP__KINEMATICS_HPP_
