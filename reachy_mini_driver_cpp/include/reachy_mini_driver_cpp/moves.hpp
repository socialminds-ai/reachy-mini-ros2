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

#ifndef REACHY_MINI_DRIVER_CPP__MOVES_HPP_
#define REACHY_MINI_DRIVER_CPP__MOVES_HPP_

// Recorded moves: loading (from a HuggingFace dataset cache or a directory)
// and evaluation. Pollen's JSON format: {"time": [...], "set_target_data":
// [{"head": 4x4, "antennas": [r, l], "body_yaw": y}, ...]} at 50 Hz.

#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "reachy_mini_driver_cpp/kinematics.hpp"

namespace reachy_mini_driver
{

constexpr const char * kDefaultDataset = "pollen-robotics/reachy-mini-emotions-library";

struct MoveSample
{
  double time;
  Pose head;
  std::array<double, 2> antennas;
  double body_yaw;
};

class RecordedMove
{
public:
  /// Parse a move JSON file.
  static RecordedMove fromFile(const std::string & path);
  explicit RecordedMove(std::vector<MoveSample> samples);

  double duration() const {return samples_.back().time;}
  /// Sample interpolated at time t (clamped to [0, duration]).
  MoveSample evaluate(double t) const;

private:
  std::vector<MoveSample> samples_;
};

class MoveLibrary
{
public:
  explicit MoveLibrary(std::string default_dataset = kDefaultDataset);
  /// Load `move` from `dataset` (default library if empty). Throws std::runtime_error.
  std::shared_ptr<const RecordedMove> get(
    const std::string & move,
    const std::string & dataset = "");
  /// Directory of a dataset: as is if it is a directory, else the HuggingFace cache snapshot.
  static std::string resolveDataset(const std::string & dataset);

private:
  std::string default_dataset_;
  std::map<std::string, std::shared_ptr<const RecordedMove>> cache_;
};

}  // namespace reachy_mini_driver

#endif  // REACHY_MINI_DRIVER_CPP__MOVES_HPP_
