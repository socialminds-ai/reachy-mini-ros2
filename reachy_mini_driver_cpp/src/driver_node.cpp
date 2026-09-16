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

#include "reachy_mini_driver_cpp/driver_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace reachy_mini_driver
{

using namespace std::chrono_literals;

const Pose kInitHeadPose = Pose::Identity();
const std::array<double, 2> kInitAntennas = {-0.1745, 0.1745};
const std::array<double, 2> kSleepAntennas = {-3.05, 3.05};
const Pose kSleepHeadPose = [] {
    Eigen::Matrix4d m;
    m << 0.911, 0.004, 0.413, -0.021,
      -0.004, 1.0, -0.001, 0.001,
      -0.413, -0.001, 0.911, -0.044,
      0.0, 0.0, 0.0, 1.0;
    Pose p;
    p.matrix() = m;
    return p;
  }();

namespace
{
geometry_msgs::msg::PoseStamped toMsg(
  const Pose & p, const rclcpp::Time & stamp,
  const std::string & frame)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame;
  msg.pose.position.x = p.translation().x();
  msg.pose.position.y = p.translation().y();
  msg.pose.position.z = p.translation().z();
  const Eigen::Quaterniond q(p.rotation());
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();
  return msg;
}

geometry_msgs::msg::TransformStamped toTransform(
  const Pose & p, const rclcpp::Time & stamp, const std::string & parent, const std::string & child)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.translation.x = p.translation().x();
  t.transform.translation.y = p.translation().y();
  t.transform.translation.z = p.translation().z();
  const Eigen::Quaterniond q(p.rotation());
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  return t;
}

Pose fromMsg(const geometry_msgs::msg::PoseStamped & msg)
{
  const auto & o = msg.pose.orientation;
  Pose p = Pose::Identity();
  p.linear() = Eigen::Quaterniond(o.w, o.x, o.y, o.z).normalized().toRotationMatrix();
  p.translation() = Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
  return p;
}

double seconds_since(std::chrono::steady_clock::time_point t0)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}
}  // namespace

DriverNode::DriverNode(std::shared_ptr<MotorBus> bus, const rclcpp::NodeOptions & options)
: rclcpp::Node("reachy_mini_driver", options),
  library_(kDefaultDataset)
{
  declare_parameter("serial_port", "/dev/ttyAMA3");
  declare_parameter("rate", 50.0);
  declare_parameter("dataset", std::string(kDefaultDataset));
  declare_parameter("wake_up_on_start", true);
  declare_parameter("sleep_on_shutdown", true);
  declare_parameter("publish_tf", true);
  declare_parameter("base_frame", "base_link");
  base_frame_ = get_parameter("base_frame").as_string();

  const auto port = get_parameter("serial_port").as_string();
  if (bus) {
    bus_ = std::move(bus);
  } else if (port == "fake") {
    bus_ = std::make_shared<FakeBus>();
  } else {
    bus_ = std::make_shared<RealBus>(port);
  }
  library_ = MoveLibrary(get_parameter("dataset").as_string());

  positions_ = bus_->readPositions();
  Joints j;
  std::copy_n(positions_.begin(), 7, j.begin());
  head_pose_ = kinematics_.fk(j, 20);
  target_pose_ = head_pose_;
  target_antennas_ = {positions_[7], positions_[8]};
  target_body_yaw_ = positions_[0];

  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/head_pose", 10);
  if (get_parameter("publish_tf").as_bool()) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/target_head_pose", 10, [this](const geometry_msgs::msg::PoseStamped & msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      goto_.reset();
      target_pose_ = fromMsg(msg);
    });
  antennas_sub_ = create_subscription<reachy_mini_msgs::msg::AntennasTarget>(
    "~/target_antennas", 10, [this](const reachy_mini_msgs::msg::AntennasTarget & msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      target_antennas_ = {msg.right, msg.left};
    });
  body_yaw_sub_ = create_subscription<std_msgs::msg::Float64>(
    "~/target_body_yaw", 10, [this](const std_msgs::msg::Float64 & msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      target_body_yaw_ = msg.data;
    });

  action_server_ = rclcpp_action::create_server<PlayMove>(
    this, "~/play_move",
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const PlayMove::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](std::shared_ptr<GoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
    [this](std::shared_ptr<GoalHandle> handle) {
      std::thread([this, handle] {executePlayMove(handle);}).detach();
    });

  bus_->setTorque(true);
  if (get_parameter("wake_up_on_start").as_bool()) {
    gotoPose(kInitHeadPose, kInitAntennas, 2.0);
  }

  const double rate = get_parameter("rate").as_double();
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate), [this] {tick();});
  RCLCPP_INFO(get_logger(), "driver ready on %s at %.0f Hz", port.c_str(), rate);
}

DriverNode::~DriverNode()
{
  move_cancelled_ = true;
}

void DriverNode::gotoPose(
  const Pose & head, const std::array<double, 2> & antennas,
  double duration)
{
  std::lock_guard<std::mutex> lock(mutex_);
  goto_ = Goto{target_pose_, head, target_antennas_, antennas, std::max(duration, 1e-3),
    std::chrono::steady_clock::now()};
}

void DriverNode::tick()
{
  const auto now = this->now();
  Positions positions;
  try {
    positions = bus_->readPositions();
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "read failed: %s", e.what());
    return;
  }

  std::optional<Positions> goals;
  Pose head_pose;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    positions_ = positions;
    Joints j;
    std::copy_n(positions.begin(), 7, j.begin());
    try {
      head_pose_ = kinematics_.fk(j, 3);
    } catch (const std::exception &) {
      // keep the previous estimate
    }
    head_pose = head_pose_;

    if (move_) {
      const double t = seconds_since(move_t0_);
      if (move_cancelled_ || t >= move_->duration()) {
        move_.reset();
        move_done_ = true;
      } else {
        const auto s = move_->evaluate(std::min(t, move_->duration() - 1e-2));
        target_pose_ = s.head;
        target_antennas_ = s.antennas;
        target_body_yaw_ = s.body_yaw;
      }
    } else if (goto_) {
      const double a = std::min(1.0, seconds_since(goto_->t0) / goto_->duration);
      target_pose_ = interpolate(goto_->start_pose, goto_->end_pose, a);
      for (int i = 0; i < 2; ++i) {
        target_antennas_[i] = goto_->start_ant[i] + (goto_->end_ant[i] - goto_->start_ant[i]) * a;
      }
      if (a >= 1.0) {
        goto_.reset();
      }
    }
    goals = computeGoals();
  }

  sensor_msgs::msg::JointState js;
  js.header.stamp = now;
  js.name.assign(kMotorNames.begin(), kMotorNames.end());
  js.position.assign(positions.begin(), positions.end());
  joint_pub_->publish(js);
  pose_pub_->publish(toMsg(head_pose, now, base_frame_));
  if (tf_broadcaster_) {
    // what robot_state_publisher cannot compute: the head (closed chain) and the rods
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.push_back(toTransform(kinematics_.headInBase(head_pose), now, base_frame_, "head"));
    Joints j;
    std::copy_n(positions.begin(), 7, j.begin());
    const auto rods = kinematics_.rodsInBase(j, head_pose);
    for (std::size_t i = 0; i < rods.size(); ++i) {
      transforms.push_back(toTransform(rods[i], now, base_frame_, "rod_" + std::to_string(i + 1)));
    }
    tf_broadcaster_->sendTransform(transforms);
  }

  if (goals) {
    try {
      bus_->writeGoalPositions(*goals);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "write failed: %s", e.what());
    }
  }
}

std::optional<Positions> DriverNode::computeGoals()
{
  const auto joints = kinematics_.ik(target_pose_, target_body_yaw_);
  if (!joints) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "target head pose is not reachable");
    return std::nullopt;
  }
  Positions goals;
  std::copy(joints->begin(), joints->end(), goals.begin());
  goals[7] = target_antennas_[0];
  goals[8] = target_antennas_[1];
  return goals;
}

void DriverNode::executePlayMove(std::shared_ptr<GoalHandle> handle)
{
  const auto goal = handle->get_goal();
  auto result = std::make_shared<PlayMove::Result>();
  std::shared_ptr<const RecordedMove> move;
  try {
    move = library_.get(goal->move, goal->dataset);
  } catch (const std::exception & e) {
    result->message = e.what();
    handle->abort(result);
    return;
  }
  RCLCPP_INFO(get_logger(), "playing '%s' (%.1f s)", goal->move.c_str(), move->duration());

  // 1. reach the first pose of the move (so the move does not start with a jump)
  const auto first = move->evaluate(0.0);
  if (goal->initial_goto_duration > 0) {
    gotoPose(first.head, first.antennas, goal->initial_goto_duration);
    std::this_thread::sleep_for(std::chrono::duration<double>(goal->initial_goto_duration));
  }
  // 2. play, ticking feedback until the loop says it is over
  {
    std::lock_guard<std::mutex> lock(mutex_);
    move_cancelled_ = false;
    move_done_ = false;
    move_ = move;
    move_t0_ = std::chrono::steady_clock::now();
  }
  auto feedback = std::make_shared<PlayMove::Feedback>();
  while (!move_done_) {
    std::this_thread::sleep_for(100ms);
    if (handle->is_canceling()) {
      move_cancelled_ = true;
      while (!move_done_) {
        std::this_thread::sleep_for(10ms);
      }
      result->message = "cancelled";
      handle->canceled(result);
      return;
    }
    const double elapsed = seconds_since(move_t0_);
    feedback->elapsed = static_cast<float>(elapsed);
    feedback->progress = static_cast<float>(std::min(1.0, elapsed / move->duration()));
    handle->publish_feedback(feedback);
  }
  result->completed = true;
  result->message = "done";
  handle->succeed(result);
}

void DriverNode::shutdown()
{
  move_cancelled_ = true;
  if (get_parameter("sleep_on_shutdown").as_bool()) {
    RCLCPP_INFO(get_logger(), "going to sleep");
    gotoPose(kSleepHeadPose, kSleepAntennas, 2.0);
    const auto deadline = std::chrono::steady_clock::now() + 2500ms;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!goto_) {
          break;
        }
      }
      tick();
      std::this_thread::sleep_for(20ms);
    }
  }
  timer_->cancel();
  bus_->setTorque(false);
}

}  // namespace reachy_mini_driver
