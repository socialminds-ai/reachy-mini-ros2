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
#include <cmath>
#include <stdexcept>
#include <vector>

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
  if (rmc_kinematics_lengths(impl_->k, &arm_length_, &rod_length_, &head_z_offset_) != 0) {
    throw std::runtime_error(rmc_last_error());
  }
  for (int i = 0; i < 6; ++i) {
    std::array<double, 16> m{};
    std::array<double, 3> a{};
    if (rmc_kinematics_leg(impl_->k, i, m.data(), a.data()) != 0) {
      throw std::runtime_error(rmc_last_error());
    }
    legs_.push_back({fromRowMajor(m), Eigen::Vector3d(a[0], a[1], a[2])});
  }
}

Pose Kinematics::headInBase(const Pose & head_pose) const
{
  Pose p = head_pose;
  p.translation().z() += head_z_offset_;
  return p;
}

std::vector<Pose> Kinematics::rodsInBase(const Joints & joints, const Pose & head_pose) const
{
  const Pose t_base_head = headInBase(head_pose);
  std::vector<Pose> rods;
  for (std::size_t i = 0; i < legs_.size(); ++i) {
    const double theta = joints[i + 1];
    const Eigen::Vector3d tip = legs_[i].t_base_motor *
      Eigen::Vector3d(arm_length_ * std::cos(theta), arm_length_ * std::sin(theta), 0.0);
    const Eigen::Vector3d anchor = t_base_head * legs_[i].anchor_head;
    Eigen::Vector3d axis = anchor - tip;
    if (axis.norm() < 1e-9) {
      axis = Eigen::Vector3d::UnitX();
    }
    Pose rod = Pose::Identity();
    rod.linear() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), axis.normalized())
      .toRotationMatrix();
    rod.translation() = tip;
    rods.push_back(rod);
  }
  return rods;
}

std::vector<double> Kinematics::rodLengths(const Joints & joints, const Pose & head_pose) const
{
  const Pose t_base_head = headInBase(head_pose);
  std::vector<double> out;
  for (std::size_t i = 0; i < legs_.size(); ++i) {
    const double theta = joints[i + 1];
    const Eigen::Vector3d tip = legs_[i].t_base_motor *
      Eigen::Vector3d(arm_length_ * std::cos(theta), arm_length_ * std::sin(theta), 0.0);
    out.push_back((t_base_head * legs_[i].anchor_head - tip).norm());
  }
  return out;
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
