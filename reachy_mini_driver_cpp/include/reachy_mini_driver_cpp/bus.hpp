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

#ifndef REACHY_MINI_DRIVER_CPP__BUS_HPP_
#define REACHY_MINI_DRIVER_CPP__BUS_HPP_

// The servo bus, behind a small interface. RealBus talks to the 9 Dynamixel
// servos through reachy_mini_c; FakeBus is a perfect servo (what you write is
// what you read), used by the tests and on laptops without the robot.

#include <array>
#include <memory>
#include <string>

struct RmcMotors;   // opaque handle from reachy_mini_c

namespace reachy_mini_driver
{

constexpr std::size_t kNumMotors = 9;
using Positions = std::array<double, kNumMotors>;
extern const std::array<const char *, kNumMotors> kMotorNames;

class MotorBus
{
public:
  virtual ~MotorBus() = default;
  virtual Positions readPositions() = 0;
  virtual void writeGoalPositions(const Positions & goals) = 0;
  virtual void setTorque(bool enabled) = 0;
};

class FakeBus : public MotorBus
{
public:
  Positions readPositions() override {return positions;}
  void writeGoalPositions(const Positions & goals) override;
  void setTorque(bool enabled) override {torque = enabled;}

  Positions positions{};
  bool torque{false};
  int writes{0};
};

class RealBus : public MotorBus
{
public:
  explicit RealBus(const std::string & serial_port);
  ~RealBus() override;
  Positions readPositions() override;
  void writeGoalPositions(const Positions & goals) override;
  void setTorque(bool enabled) override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace reachy_mini_driver

#endif  // REACHY_MINI_DRIVER_CPP__BUS_HPP_
