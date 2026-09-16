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

#ifndef REACHY_MINI_DRIVER_CPP__DRIVER_NODE_HPP_
#define REACHY_MINI_DRIVER_CPP__DRIVER_NODE_HPP_

// The driver node: a 50 Hz control loop around the servo bus. Every tick:
//   1. read the 9 motor positions, publish JointState + head pose (FK);
//   2. decide the target: the move being played, the goto in progress, or
//      the last targets received on the topics;
//   3. inverse kinematics -> 9 goal positions -> write them.

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "reachy_mini_driver_cpp/bus.hpp"
#include "reachy_mini_driver_cpp/kinematics.hpp"
#include "reachy_mini_driver_cpp/moves.hpp"
#include "reachy_mini_msgs/action/play_move.hpp"
#include "reachy_mini_msgs/msg/antennas_target.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace reachy_mini_driver
{

class DriverNode : public rclcpp::Node
{
public:
  using PlayMove = reachy_mini_msgs::action::PlayMove;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PlayMove>;

  /// `bus` overrides the serial_port parameter (tests).
  explicit DriverNode(
    std::shared_ptr<MotorBus> bus = nullptr,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DriverNode() override;

  /// Smoothly move to a head pose and antenna positions.
  void gotoPose(const Pose & head, const std::array<double, 2> & antennas, double duration);
  /// Go to the sleep pose and release the motors (call before destroying).
  void shutdown();

  std::shared_ptr<MotorBus> bus() {return bus_;}

private:
  struct Goto
  {
    Pose start_pose, end_pose;
    std::array<double, 2> start_ant, end_ant;
    double duration;
    std::chrono::steady_clock::time_point t0;
  };

  void tick();
  std::optional<Positions> computeGoals();
  void executePlayMove(std::shared_ptr<GoalHandle> handle);

  std::shared_ptr<MotorBus> bus_;
  Kinematics kinematics_;
  MoveLibrary library_;

  std::mutex mutex_;
  Positions positions_{};
  Pose head_pose_{Pose::Identity()};
  Pose target_pose_{Pose::Identity()};
  std::array<double, 2> target_antennas_{};
  double target_body_yaw_{0.0};
  std::optional<Goto> goto_;
  std::shared_ptr<const RecordedMove> move_;
  std::chrono::steady_clock::time_point move_t0_;
  std::atomic<bool> move_cancelled_{false};
  std::atomic<bool> move_done_{true};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string base_frame_{"base_link"};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<reachy_mini_msgs::msg::AntennasTarget>::SharedPtr antennas_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr body_yaw_sub_;
  rclcpp_action::Server<PlayMove>::SharedPtr action_server_;
};

extern const Pose kInitHeadPose;
extern const Pose kSleepHeadPose;
extern const std::array<double, 2> kInitAntennas;
extern const std::array<double, 2> kSleepAntennas;

}  // namespace reachy_mini_driver

#endif  // REACHY_MINI_DRIVER_CPP__DRIVER_NODE_HPP_
