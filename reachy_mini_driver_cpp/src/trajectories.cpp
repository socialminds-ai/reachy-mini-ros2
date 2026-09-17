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

#include "reachy_mini_driver_cpp/trajectories.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace reachy_mini_driver
{

Targets CartesianMotion::evaluate(double t) const
{
  const MoveSample s = move_->evaluate(t);
  Targets out;
  out.head = s.head;
  out.antennas = s.antennas;
  out.body_yaw = s.body_yaw;
  return out;
}

JointMotion::JointMotion(std::vector<std::string> names, std::vector<Point> points)
: points_(std::move(points))
{
  if (names.empty() || points_.empty()) {
    throw std::invalid_argument("a joint trajectory needs at least one joint and one point");
  }
  for (const auto & name : names) {
    const auto it = std::find(kMotorNames.begin(), kMotorNames.end(), name);
    if (it == kMotorNames.end()) {
      throw std::invalid_argument("unknown joint '" + name + "'");
    }
    indices_.push_back(static_cast<std::size_t>(it - kMotorNames.begin()));
  }
  double last = -1.0;
  for (const auto & p : points_) {
    if (p.positions.size() != names.size()) {
      throw std::invalid_argument("a point does not have one position per joint");
    }
    if (p.time <= last) {
      throw std::invalid_argument("time_from_start must be strictly increasing");
    }
    last = p.time;
  }
}

Targets JointMotion::evaluate(double t) const
{
  t = std::clamp(t, 0.0, points_.back().time);
  // segment [a, b] containing t; before the first point, interpolate from the first point itself
  std::size_t b = 0;
  while (b < points_.size() && points_[b].time < t) {
    ++b;
  }
  const Point & pb = points_[std::min(b, points_.size() - 1)];
  const Point & pa = (b == 0) ? pb : points_[b - 1];
  const double span = pb.time - pa.time;
  const double alpha = span > 0.0 ? (t - pa.time) / span : 1.0;
  Targets out;
  std::map<std::size_t, double> joints;
  for (std::size_t i = 0; i < indices_.size(); ++i) {
    joints[indices_[i]] = pa.positions[i] + (pb.positions[i] - pa.positions[i]) * alpha;
  }
  out.joints = joints;
  return out;
}

}  // namespace reachy_mini_driver
