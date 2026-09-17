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

// Tests of the driver on the fake bus: loop, TF, targets and the ROS4HRI skills.
#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "interaction_skills/action/look_at.hpp"
#include "motions_skills/action/execute_cartesian_trajectory.hpp"
#include "motions_skills/action/execute_joint_trajectory.hpp"
#include "motions_skills/action/replay_motion.hpp"
#include "motions_skills/srv/list_motions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "reachy_mini_driver_cpp/bus.hpp"
#include "reachy_mini_driver_cpp/driver_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_skills/msg/result.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using namespace std::chrono_literals;
using reachy_mini_driver::DriverNode;
using reachy_mini_driver::FakeBus;
using ReplayMotion = motions_skills::action::ReplayMotion;
using ExecuteCartesianTrajectory = motions_skills::action::ExecuteCartesianTrajectory;
using ExecuteJointTrajectory = motions_skills::action::ExecuteJointTrajectory;
using LookAt = interaction_skills::action::LookAt;
using ListMotions = motions_skills::srv::ListMotions;
using Result = std_skills::msg::Result;

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
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("dataset", std::string(TEST_MOVES_DIR)),
        rclcpp::Parameter("glance_duration", 0.5)});
    node = std::make_shared<DriverNode>(bus, options);
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 3);
    executor->add_node(node);
    thread = std::thread([this] {executor->spin();});
    client_node = std::make_shared<rclcpp::Node>("test_client");
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
  /// Send an action goal and wait for its result; returns the std_skills error code.
  template<typename Action>
  uint8_t run(const std::string & name, const typename Action::Goal & goal, int & feedbacks)
  {
    auto client = rclcpp_action::create_client<Action>(client_node, name);
    if (!client->wait_for_action_server(5s)) {
      return 250;
    }
    typename rclcpp_action::Client<Action>::SendGoalOptions opts;
    opts.feedback_callback = [&feedbacks](auto, auto) {++feedbacks;};
    auto goal_future = client->async_send_goal(goal, opts);
    if (rclcpp::spin_until_future_complete(client_node, goal_future, 5s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return 251;
    }
    auto handle = goal_future.get();
    if (!handle) {
      return 252;   // rejected
    }
    auto result_future = client->async_get_result(handle);
    if (rclcpp::spin_until_future_complete(client_node, result_future, 15s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return 253;
    }
    last_message = result_future.get().result->result.error_msg;
    return result_future.get().result->result.error_code;
  }
  /// Wait until the wake-up goto is over.
  void awake() {std::this_thread::sleep_for(2200ms);}

  std::shared_ptr<FakeBus> bus;
  std::shared_ptr<DriverNode> node;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::thread thread;
  std::shared_ptr<rclcpp::Node> client_node;
  std::string last_message;
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
  int count = 0;
  sensor_msgs::msg::JointState last;
  auto sub = client_node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, [&](const sensor_msgs::msg::JointState & m) {++count; last = m;});
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline && count < 5) {
    rclcpp::spin_some(client_node);
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GE(count, 5);
  EXPECT_EQ(last.name[0], "body_yaw");
  EXPECT_EQ(last.position.size(), 9u);
}

TEST_F(DriverTest, TfFramesAreBroadcast)
{
  tf2_msgs::msg::TFMessage last;
  int count = 0;
  auto sub = client_node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", 10, [&](const tf2_msgs::msg::TFMessage & m) {++count; last = m;});
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline && count < 3) {
    rclcpp::spin_some(client_node);
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GE(count, 3);
  ASSERT_EQ(last.transforms.size(), 7u);
  EXPECT_EQ(last.transforms[0].child_frame_id, "head");
  EXPECT_GT(last.transforms[0].transform.translation.z, 0.15);
}

TEST_F(DriverTest, TargetHeadPoseMovesThePlatform)
{
  awake();
  const auto before = bus->positions;
  auto pub = client_node->create_publisher<geometry_msgs::msg::PoseStamped>(
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

// --- skills -----------------------------------------------------------------

TEST_F(DriverTest, ListMotions)
{
  auto client = client_node->create_client<ListMotions>("/skill/list_motions");
  ASSERT_TRUE(client->wait_for_service(5s));
  auto future = client->async_send_request(std::make_shared<ListMotions::Request>());
  ASSERT_EQ(rclcpp::spin_until_future_complete(client_node, future, 5s),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto response = future.get();
  EXPECT_EQ(response->result.error_code, Result::ROS_ENOERR);
  EXPECT_EQ(response->motions, std::vector<std::string>{"nod"});
}

TEST_F(DriverTest, ReplayMotionPlaysAndReports)
{
  awake();
  ReplayMotion::Goal goal;
  goal.meta.caller = "test";
  goal.name = "nod";
  int feedbacks = 0;
  bool wiggled = false;
  auto antenna_down = [this] {return std::fabs(bus->positions[7] + 0.3) < 0.1;};
  std::thread checker([&] {wiggled = waitFor(antenna_down, 4000ms);});
  const uint8_t code = run<ReplayMotion>("/skill/replay_motion", goal, feedbacks);
  checker.join();
  EXPECT_EQ(code, Result::ROS_ENOERR);
  EXPECT_TRUE(wiggled);   // the nod moves the right antenna to -0.3 rad at its peak
  EXPECT_GT(feedbacks, 0);
}

TEST_F(DriverTest, ReplayMotionUnknownNameListsTheMotions)
{
  ReplayMotion::Goal goal;
  goal.name = "does_not_exist";
  int feedbacks = 0;
  EXPECT_EQ(run<ReplayMotion>("/skill/replay_motion", goal, feedbacks), Result::ROS_EBADMSG);
  EXPECT_NE(last_message.find("nod"), std::string::npos);
  goal.name = "";
  EXPECT_EQ(run<ReplayMotion>("/skill/replay_motion", goal, feedbacks), Result::ROS_EBADMSG);
}

TEST_F(DriverTest, HigherPriorityPreemptsLowerIsRejected)
{
  awake();
  auto client = rclcpp_action::create_client<ReplayMotion>(client_node, "/skill/replay_motion");
  ASSERT_TRUE(client->wait_for_action_server(5s));
  ReplayMotion::Goal low;
  low.name = "nod";
  low.meta.priority = 10;
  auto low_future = client->async_send_goal(low);
  ASSERT_EQ(rclcpp::spin_until_future_complete(client_node, low_future, 5s),
    rclcpp::FutureReturnCode::SUCCESS);
  auto low_handle = low_future.get();
  ASSERT_TRUE(low_handle);
  auto low_result = client->async_get_result(low_handle);
  std::this_thread::sleep_for(700ms);   // initial goto (0.5 s) + a bit of playback
  // lower priority: rejected
  ReplayMotion::Goal lower = low;
  lower.meta.priority = 5;
  auto lower_future = client->async_send_goal(lower);
  ASSERT_EQ(rclcpp::spin_until_future_complete(client_node, lower_future, 5s),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_FALSE(lower_future.get());
  // same or higher priority: preempts, the first goal ends with ROS_EINTR
  ReplayMotion::Goal high = low;
  high.meta.priority = 10;
  int feedbacks = 0;
  EXPECT_EQ(run<ReplayMotion>("/skill/replay_motion", high, feedbacks), Result::ROS_ENOERR);
  ASSERT_EQ(rclcpp::spin_until_future_complete(client_node, low_result, 5s),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(low_result.get().result->result.error_code, Result::ROS_EINTR);
}

TEST_F(DriverTest, CartesianTrajectoryMovesTheHead)
{
  awake();
  const auto before = bus->positions;
  ExecuteCartesianTrajectory::Goal goal;
  goal.trajectory.header.frame_id = "base_link";
  goal.trajectory.tracked_frame = "head";
  for (int i = 0; i < 2; ++i) {
    moveit_msgs::msg::CartesianTrajectoryPoint p;
    p.time_from_start = rclcpp::Duration::from_seconds(0.5 * (i + 1));
    p.point.pose.orientation.w = 1.0;
    p.point.pose.position.z = 0.177 + 0.02;   // in base_link: rest height + 2 cm
    goal.trajectory.points.push_back(p);
  }
  int feedbacks = 0;
  EXPECT_EQ(run<ExecuteCartesianTrajectory>("/skill/execute_cartesian_trajectory", goal, feedbacks),
    Result::ROS_ENOERR);
  double d = 0;
  for (int i = 1; i < 7; ++i) {
    d = std::max(d, std::fabs(bus->positions[i] - before[i]));
                                                                                         }
  EXPECT_GT(d, 0.05);
  // wrong frame: bad message
  goal.trajectory.tracked_frame = "camera";
  EXPECT_EQ(run<ExecuteCartesianTrajectory>("/skill/execute_cartesian_trajectory", goal, feedbacks),
    Result::ROS_EBADMSG);
}

TEST_F(DriverTest, JointTrajectoryMovesTheAntennasAndProtectsThePlatform)
{
  awake();
  ExecuteJointTrajectory::Goal goal;
  goal.trajectory.joint_names = {"right_antenna", "left_antenna"};
  trajectory_msgs::msg::JointTrajectoryPoint p;
  p.positions = {-1.0, 1.0};
  p.time_from_start = rclcpp::Duration::from_seconds(0.5);
  goal.trajectory.points.push_back(p);
  int feedbacks = 0;
  EXPECT_EQ(run<ExecuteJointTrajectory>("/skill/execute_joint_trajectory", goal, feedbacks),
    Result::ROS_ENOERR);
  EXPECT_NEAR(bus->positions[7], -1.0, 0.05);
  EXPECT_NEAR(bus->positions[8], 1.0, 0.05);
  // Stewart joints are refused in safe mode, accepted otherwise
  goal.trajectory.joint_names = {"stewart_1"};
  goal.trajectory.points[0].positions = {0.7};
  EXPECT_EQ(run<ExecuteJointTrajectory>("/skill/execute_joint_trajectory", goal, feedbacks),
    Result::ROS_EBADMSG);
  goal.safe_mode = false;
  EXPECT_EQ(run<ExecuteJointTrajectory>("/skill/execute_joint_trajectory", goal, feedbacks),
    Result::ROS_ENOERR);
  EXPECT_NEAR(bus->positions[1], 0.7, 0.05);
  // unknown joint
  goal.trajectory.joint_names = {"elbow"};
  EXPECT_EQ(run<ExecuteJointTrajectory>("/skill/execute_joint_trajectory", goal, feedbacks),
    Result::ROS_EBADMSG);
}

TEST_F(DriverTest, LookAtTracksGlancesAndResets)
{
  awake();
  // a point on the left of the robot (in base_link, no TF needed): the head must yaw left
  LookAt::Goal goal;
  goal.target.header.frame_id = "base_link";
  goal.target.point.x = 0.5;
  goal.target.point.y = 0.5;
  goal.target.point.z = 0.177;
  int feedbacks = 0;
  EXPECT_EQ(run<LookAt>("/skill/look_at", goal, feedbacks), Result::ROS_ENOERR);
  // the body yaw follows (automatic body yaw) or the platform joints change: any of both
  auto turned = [this] {
      return bus->positions[0] > 0.2 || std::fabs(bus->positions[1] - 0.6265) > 0.05;
    };
  EXPECT_TRUE(waitFor(turned, 3000ms));
  // reset: back straight
  LookAt::Goal reset;
  reset.policy = LookAt::Goal::RESET;
  EXPECT_EQ(run<LookAt>("/skill/look_at", reset, feedbacks), Result::ROS_ENOERR);
  EXPECT_TRUE(waitFor([this] {return std::fabs(bus->positions[0]) < 0.05;}, 3000ms));
  // glance: looks, then comes back within glance_duration (0.5 s) + a goto
  LookAt::Goal glance = goal;
  glance.policy = LookAt::Goal::GLANCE;
  EXPECT_EQ(run<LookAt>("/skill/look_at", glance, feedbacks), Result::ROS_ENOERR);
  EXPECT_TRUE(waitFor([this] {return std::fabs(bus->positions[0]) < 0.05;}, 3000ms));
  // unsupported policy and missing target
  LookAt::Goal social;
  social.policy = LookAt::Goal::SOCIAL;
  EXPECT_EQ(run<LookAt>("/skill/look_at", social, feedbacks), Result::ROS_ENOTSUP);
  LookAt::Goal empty;
  EXPECT_EQ(run<LookAt>("/skill/look_at", empty, feedbacks), Result::ROS_EBADMSG);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  // a private domain: colcon runs the packages' tests in parallel, and the Python driver test
  // starts the same node name and skill servers
  setenv("ROS_DOMAIN_ID", "72", 0);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
