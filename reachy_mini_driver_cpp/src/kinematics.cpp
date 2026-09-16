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

#include "reachy_mini_driver_cpp/kinematics.hpp"

#include <reachy_mini_c/reachy_mini_c.h>

#include <array>
#include <stdexcept>

namespace reachy_mini_driver
{

struct Kinematics::Impl
{
  RmcKinematics * k{nullptr};
};

namespace
{
std::array<double, 16> toRowMajor(const Pose & p)
{
  std::array<double, 16> m{};
  const Eigen::Matrix4d mat = p.matrix();
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      m[r * 4 + c] = mat(r, c);
    }
  }
  return m;
}

Pose fromRowMajor(const std::array<double, 16> & m)
{
  Eigen::Matrix4d mat;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      mat(r, c) = m[r * 4 + c];
    }
  }
  Pose p;
  p.matrix() = mat;
  return p;
}
}  // namespace

Kinematics::Kinematics()
: impl_(std::make_unique<Impl>())
{
  impl_->k = rmc_kinematics_new();
  if (!impl_->k) {
    throw std::runtime_error(rmc_last_error());
  }
}

Kinematics::~Kinematics()
{
  rmc_kinematics_free(impl_->k);
}

std::optional<Joints> Kinematics::ik(const Pose & head_pose, double body_yaw)
{
  Joints j{};
  const auto m = toRowMajor(head_pose);
  if (rmc_kinematics_ik(impl_->k, m.data(), body_yaw, j.data()) != 0) {
    return std::nullopt;
  }
  return j;
}

Pose Kinematics::fk(const Joints & joints, int iterations)
{
  std::array<double, 16> m{};
  if (rmc_kinematics_fk(impl_->k, joints.data(), iterations, m.data()) != 0) {
    throw std::runtime_error(rmc_last_error());
  }
  return fromRowMajor(m);
}

void Kinematics::resetFk(const Pose & head_pose)
{
  const auto m = toRowMajor(head_pose);
  rmc_kinematics_reset_fk(impl_->k, m.data());
}

Pose interpolate(const Pose & a, const Pose & b, double t)
{
  const Eigen::Quaterniond qa(a.rotation()), qb(b.rotation());
  Pose p = Pose::Identity();
  p.linear() = qa.slerp(t, qb).toRotationMatrix();
  p.translation() = a.translation() + (b.translation() - a.translation()) * t;
  return p;
}

}  // namespace reachy_mini_driver
