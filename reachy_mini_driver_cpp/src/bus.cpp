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

#include "reachy_mini_driver_cpp/bus.hpp"

#include <reachy_mini_c/reachy_mini_c.h>

#include <stdexcept>
#include <string>

namespace reachy_mini_driver
{

const std::array<const char *, kNumMotors> kMotorNames = {
  "body_yaw", "stewart_1", "stewart_2", "stewart_3", "stewart_4", "stewart_5", "stewart_6",
  "right_antenna", "left_antenna"};

void FakeBus::writeGoalPositions(const Positions & goals)
{
  ++writes;
  if (torque) {
    positions = goals;
  }
}

struct RealBus::Impl
{
  RmcMotors * motors{nullptr};
};

RealBus::RealBus(const std::string & serial_port)
: impl_(std::make_unique<Impl>())
{
  impl_->motors = rmc_motors_open(serial_port.c_str());
  if (!impl_->motors) {
    throw std::runtime_error(rmc_last_error());
  }
}

RealBus::~RealBus()
{
  rmc_motors_close(impl_->motors);
}

Positions RealBus::readPositions()
{
  Positions p{};
  if (rmc_motors_read_positions(impl_->motors, p.data()) != 0) {
    throw std::runtime_error(rmc_last_error());
  }
  return p;
}

void RealBus::writeGoalPositions(const Positions & goals)
{
  if (rmc_motors_write_goal_positions(impl_->motors, goals.data()) != 0) {
    throw std::runtime_error(rmc_last_error());
  }
}

void RealBus::setTorque(bool enabled)
{
  if (rmc_motors_set_torque(impl_->motors, enabled ? 1 : 0) != 0) {
    throw std::runtime_error(rmc_last_error());
  }
}

}  // namespace reachy_mini_driver
