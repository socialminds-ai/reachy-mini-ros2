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

// Tests of the driver on the fake bus: loop, targets, play_move action.
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "reachy_mini_driver_cpp/bus.hpp"
#include "reachy_mini_driver_cpp/driver_node.hpp"
#include "reachy_mini_msgs/action/play_move.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using namespace std::chrono_literals;
using reachy_mini_driver::DriverNode;
using reachy_mini_driver::FakeBus;
using PlayMove = reachy_mini_msgs::action::PlayMove;

namespace
{
template<typename Pred>
bool waitFor(Pred pred, std::chrono::milliseconds timeout = 5000ms)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) {
      return true;
    }
    std::this_thread::sleep_for(20ms);
  }
  return false;
}

class DriverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    bus = std::make_shared<FakeBus>();
    node = std::make_shared<DriverNode>(bus);
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(),
        3);
    executor->add_node(node);
    thread = std::thread([this] {executor->spin();});
  }
  void TearDown() override
  {
    node->set_parameter(rclcpp::Parameter("sleep_on_shutdown", false));
    node->shutdown();
    executor->cancel();
    thread.join();
    executor.reset();
    node.reset();
  }
  std::shared_ptr<FakeBus> bus;
  std::shared_ptr<DriverNode> node;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::thread thread;
};
}  // namespace

TEST_F(DriverTest, TorqueEnabledAndLoopWrites)
{
  EXPECT_TRUE(bus->torque);
  EXPECT_TRUE(waitFor([this] {return bus->writes > 10;}));
}

TEST_F(DriverTest, WakeUpReachesInitPose)
{
  EXPECT_TRUE(
    waitFor(
      [this] {
        return std::fabs(bus->positions[7] - reachy_mini_driver::kInitAntennas[0]) < 1e-3 &&
               std::fabs(bus->positions[8] - reachy_mini_driver::kInitAntennas[1]) < 1e-3;
      }, 4000ms));
  EXPECT_NEAR(bus->positions[0], 0.0, 1e-6);
}

TEST_F(DriverTest, JointStatesArePublished)
{
  auto listener = std::make_shared<rclcpp::Node>("listener");
  int count = 0;
  sensor_msgs::msg::JointState last;
  auto sub = listener->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, [&](const sensor_msgs::msg::JointState & m) {++count; last = m;});
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline && count < 5) {
    rclcpp::spin_some(listener);
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GE(count, 5);
  EXPECT_EQ(last.name[0], "body_yaw");
  EXPECT_EQ(last.position.size(), 9u);
}

TEST_F(DriverTest, TfFramesAreBroadcast)
{
  auto listener = std::make_shared<rclcpp::Node>("tf_listener");
  tf2_msgs::msg::TFMessage last;
  int count = 0;
  auto sub = listener->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", 10, [&](const tf2_msgs::msg::TFMessage & m) {++count; last = m;});
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline && count < 3) {
    rclcpp::spin_some(listener);
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GE(count, 3);
  ASSERT_EQ(last.transforms.size(), 7u);
  EXPECT_EQ(last.transforms[0].child_frame_id, "head");
  EXPECT_EQ(last.transforms[0].header.frame_id, "base_link");
  EXPECT_GT(last.transforms[0].transform.translation.z, 0.15);
  EXPECT_EQ(last.transforms[6].child_frame_id, "rod_6");
}

TEST_F(DriverTest, TargetHeadPoseMovesThePlatform)
{
  std::this_thread::sleep_for(2200ms);   // let the wake-up finish
  const auto before = bus->positions;
  auto pub_node = std::make_shared<rclcpp::Node>("pub");
  auto pub = pub_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/reachy_mini_driver/target_head_pose", 10);
  ASSERT_TRUE(waitFor([&pub] {return pub->get_subscription_count() == 1;}));
  geometry_msgs::msg::PoseStamped msg;
  msg.pose.position.z = 0.02;
  msg.pose.orientation.w = 1.0;
  pub->publish(msg);
  EXPECT_TRUE(
    waitFor(
      [&] {
        double d = 0;
        for (int i = 1; i < 7; ++i) {
          d = std::max(d, std::fabs(bus->positions[i] - before[i]));
        }
        return d > 0.05;
      }));
}

TEST_F(DriverTest, PlayMoveAction)
{
  std::this_thread::sleep_for(2200ms);
  auto client_node = std::make_shared<rclcpp::Node>("client");
  auto client = rclcpp_action::create_client<PlayMove>(client_node,
    "/reachy_mini_driver/play_move");
  ASSERT_TRUE(client->wait_for_action_server(5s));
  PlayMove::Goal goal;
  goal.dataset = TEST_MOVES_DIR;
  goal.move = "nod";
  goal.initial_goto_duration = 0.2f;
  int feedbacks = 0;
  rclcpp_action::Client<PlayMove>::SendGoalOptions opts;
  opts.feedback_callback = [&feedbacks](auto, auto) {++feedbacks;};
  auto goal_future = client->async_send_goal(goal, opts);
  ASSERT_EQ(rclcpp::spin_until_future_complete(client_node, goal_future, 5s),
    rclcpp::FutureReturnCode::SUCCESS);
  auto handle = goal_future.get();
  ASSERT_TRUE(handle);
  auto result_future = client->async_get_result(handle);
  // the antennas must wiggle while the 1 s move plays
  EXPECT_TRUE(waitFor([this] {return std::fabs(bus->positions[7] + 0.3) < 0.1;}, 2000ms));
  ASSERT_EQ(rclcpp::spin_until_future_complete(client_node, result_future, 5s),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(result.result->completed);
  EXPECT_GT(feedbacks, 0);
}

TEST_F(DriverTest, PlayMoveUnknownIsAborted)
{
  auto client_node = std::make_shared<rclcpp::Node>("client2");
  auto client = rclcpp_action::create_client<PlayMove>(client_node,
    "/reachy_mini_driver/play_move");
  ASSERT_TRUE(client->wait_for_action_server(5s));
  PlayMove::Goal goal;
  goal.dataset = TEST_MOVES_DIR;
  goal.move = "does_not_exist";
  auto goal_future = client->async_send_goal(goal);
  ASSERT_EQ(rclcpp::spin_until_future_complete(client_node, goal_future, 5s),
    rclcpp::FutureReturnCode::SUCCESS);
  auto result_future = client->async_get_result(goal_future.get());
  ASSERT_EQ(rclcpp::spin_until_future_complete(client_node, result_future, 5s),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_NE(result_future.get().result->message.find("not found"), std::string::npos);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
