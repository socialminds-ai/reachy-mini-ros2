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

#ifndef REACHY_MINI_DRIVER_CPP__TRAJECTORIES_HPP_
#define REACHY_MINI_DRIVER_CPP__TRAJECTORIES_HPP_

// What the control loop plays: anything that gives targets as a function of
// time. Recorded moves and Cartesian trajectories give head pose, antennas
// and body yaw (then inverse kinematics runs); joint trajectories give joint
// goals directly (no kinematics).

#include <array>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "reachy_mini_driver_cpp/bus.hpp"
#include "reachy_mini_driver_cpp/kinematics.hpp"
#include "reachy_mini_driver_cpp/moves.hpp"

namespace reachy_mini_driver
{

/// Targets at one instant. Unset fields keep their current value.
struct Targets
{
  std::optional<Pose> head;
  std::optional<std::array<double, 2>> antennas;
  std::optional<double> body_yaw;
  /// Joint goals by motor index (see kMotorNames); when set, the loop writes
  /// them as is, bypassing kinematics.
  std::optional<std::map<std::size_t, double>> joints;
};

class Motion
{
public:
  virtual ~Motion() = default;
  virtual double duration() const = 0;
  virtual Targets evaluate(double t) const = 0;
};

/// A recorded move (or a Cartesian trajectory converted to one).
class CartesianMotion : public Motion
{
public:
  explicit CartesianMotion(std::shared_ptr<const RecordedMove> move)
  : move_(std::move(move)) {}
  double duration() const override {return move_->duration();}
  Targets evaluate(double t) const override;

private:
  std::shared_ptr<const RecordedMove> move_;
};

/// Joint-space motion: per-joint positions at increasing times, linearly interpolated.
class JointMotion : public Motion
{
public:
  struct Point
  {
    double time;
    std::vector<double> positions;   // one per joint of `names`
  };
  /// `names` are motor names (kMotorNames). Throws std::invalid_argument on bad input.
  JointMotion(std::vector<std::string> names, std::vector<Point> points);
  double duration() const override {return points_.back().time;}
  Targets evaluate(double t) const override;
  const std::vector<std::size_t> & indices() const {return indices_;}

private:
  std::vector<std::size_t> indices_;
  std::vector<Point> points_;
};

}  // namespace reachy_mini_driver

#endif  // REACHY_MINI_DRIVER_CPP__TRAJECTORIES_HPP_
