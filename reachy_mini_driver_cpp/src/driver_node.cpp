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
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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
constexpr double kMaxLookPitch = 35.0 * M_PI / 180.0;   // the head cannot tilt further

geometry_msgs::msg::PoseStamped toMsg(
  const Pose & p, const rclcpp::Time & stamp, const std::string & frame)
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

Pose fromMsg(const geometry_msgs::msg::Pose & msg)
{
  const auto & o = msg.orientation;
  Pose p = Pose::Identity();
  p.linear() = Eigen::Quaterniond(o.w, o.x, o.y, o.z).normalized().toRotationMatrix();
  p.translation() = Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
  return p;
}

double seconds_since(std::chrono::steady_clock::time_point t0)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

std_skills::msg::Result makeResult(uint8_t code, const std::string & msg = "")
{
  std_skills::msg::Result r;
  r.error_code = code;
  r.error_msg = msg;
  return r;
}

std::string join(const std::vector<std::string> & v)
{
  std::string out;
  for (const auto & s : v) {
    out += (out.empty() ? "" : ", ") + s;
  }
  return out;
}
}  // namespace

DriverNode::DriverNode(std::shared_ptr<MotorBus> bus, const rclcpp::NodeOptions & options)
: rclcpp::Node("reachy_mini_driver", options),
  library_(kDefaultDataset)
{
  declare_parameter("serial_port", "/dev/ttyAMA3");
  declare_parameter("rate", 50.0);
  declare_parameter("dataset", std::string(kDefaultDataset));
  declare_parameter("extra_datasets", std::vector<std::string>{});
  declare_parameter("wake_up_on_start", true);
  declare_parameter("sleep_on_shutdown", true);
  declare_parameter("publish_tf", true);
  declare_parameter("base_frame", "base_link");
  declare_parameter("skills_namespace", "/skill");
  declare_parameter("glance_duration", 2.0);
  declare_parameter("motion_initial_goto", 0.5);
  // speed limits: smooth ones for reactive targets and gotos, safety caps during motions
  auto declare_limit = [this](const std::string & part, double velocity, double acceleration,
    double safety) {
      return std::make_pair(
        SpeedLimit{declare_parameter("max_speed." + part, velocity),
          declare_parameter("max_acceleration." + part, acceleration)},
        SpeedLimit{declare_parameter("safety_max_speed." + part, safety)});
    };
  std::tie(smooth_limits_.head_rotation, safety_limits_.head_rotation) =
    declare_limit("head_rotation", 1.5, 6.0, 10.0);           // rad/s, rad/s², rad/s
  std::tie(smooth_limits_.head_translation, safety_limits_.head_translation) =
    declare_limit("head_translation", 0.1, 0.5, 0.5);         // m/s, m/s², m/s
  std::tie(smooth_limits_.antennas, safety_limits_.antennas) =
    declare_limit("antennas", 4.0, 20.0, 15.0);               // rad/s, rad/s², rad/s
  std::tie(smooth_limits_.body_yaw, safety_limits_.body_yaw) =
    declare_limit("body_yaw", 1.5, 6.0, 4.0);                 // rad/s, rad/s², rad/s
  base_frame_ = get_parameter("base_frame").as_string();
  extra_datasets_ = get_parameter("extra_datasets").as_string_array();
  glance_duration_ = get_parameter("glance_duration").as_double();
  motion_initial_goto_ = get_parameter("motion_initial_goto").as_double();

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
  last_goals_ = positions_;
  Joints j;
  std::copy_n(positions_.begin(), 7, j.begin());
  head_pose_ = kinematics_.fk(j, 20);
  target_pose_ = head_pose_;
  target_antennas_ = {positions_[7], positions_[8]};
  target_body_yaw_ = positions_[0];
  limiter_.reset(targets());
  last_tick_ = std::chrono::steady_clock::now();

  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/head_pose", 10);
  if (get_parameter("publish_tf").as_bool()) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/target_head_pose", 10, [this](const geometry_msgs::msg::PoseStamped & msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      goto_.reset();
      gaze_.reset();
      target_pose_ = fromMsg(msg.pose);
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

  // --- skills ---------------------------------------------------------------
  const std::string ns = get_parameter("skills_namespace").as_string();
  auto motion_goal_cb = [this](uint8_t priority) {
      return acceptMotionGoal(priority) ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
             rclcpp_action::GoalResponse::REJECT;
    };
  replay_server_ = rclcpp_action::create_server<ReplayMotion>(
    this, ns + "/replay_motion",
    [motion_goal_cb](const rclcpp_action::GoalUUID &, std::shared_ptr<const ReplayMotion::Goal> g) {
      return motion_goal_cb(g->meta.priority);
    },
    [](std::shared_ptr<rclcpp_action::ServerGoalHandle<ReplayMotion>>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<ReplayMotion>> handle) {
      std::thread([this, handle] {executeReplayMotion(handle);}).detach();
    });
  cartesian_server_ = rclcpp_action::create_server<ExecuteCartesianTrajectory>(
    this, ns + "/execute_cartesian_trajectory",
    [motion_goal_cb](const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteCartesianTrajectory::Goal> g) {
      return motion_goal_cb(g->meta.priority);
    },
    [](std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteCartesianTrajectory>>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteCartesianTrajectory>> handle) {
      std::thread([this, handle] {executeCartesianTrajectory(handle);}).detach();
    });
  joint_server_ = rclcpp_action::create_server<ExecuteJointTrajectory>(
    this, ns + "/execute_joint_trajectory",
    [motion_goal_cb](const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteJointTrajectory::Goal> g) {
      return motion_goal_cb(g->meta.priority);
    },
    [](std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteJointTrajectory>>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteJointTrajectory>> handle) {
      std::thread([this, handle] {executeJointTrajectory(handle);}).detach();
    });
  look_at_server_ = rclcpp_action::create_server<LookAt>(
    this, ns + "/look_at",
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const LookAt::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](std::shared_ptr<rclcpp_action::ServerGoalHandle<LookAt>>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<LookAt>> handle) {
      std::thread([this, handle] {executeLookAt(handle);}).detach();
    });
  list_service_ = create_service<ListMotions>(
    ns + "/list_motions",
    [this](const std::shared_ptr<ListMotions::Request> req,
    std::shared_ptr<ListMotions::Response> res) {listMotions(req, res);});

  bus_->setTorque(true);
  if (get_parameter("wake_up_on_start").as_bool()) {
    gotoPose(kInitHeadPose, kInitAntennas, 2.0);
  }

  const double rate = get_parameter("rate").as_double();
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate), [this] {tick();});
  RCLCPP_INFO(
    get_logger(), "driver ready on %s at %.0f Hz; skills under %s", port.c_str(), rate,
    ns.c_str());
}

DriverNode::~DriverNode()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (current_) {
    current_->cancelled = true;
    current_->done = true;
  }
}

double DriverNode::gotoPose(
  const Pose & head, const std::array<double, 2> & antennas, double duration,
  std::optional<double> body_yaw)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return startGoto(head, antennas, body_yaw.value_or(target_body_yaw_), duration);
}

double DriverNode::startGoto(
  const Pose & head, const std::array<double, 2> & antennas, double body_yaw, double duration)
{
  // from where the robot is commanded to be, which lags the targets while limited
  const Command & from = limiter_.command();
  duration = gotoDuration(from, Command{head, antennas, body_yaw}, smooth_limits_, duration);
  goto_ = Goto{from.head, head, from.antennas, antennas, from.body_yaw, body_yaw,
    std::max(duration, 1e-3), std::chrono::steady_clock::now()};
  return duration;
}

// ---------------------------------------------------------------------------
// the loop
// ---------------------------------------------------------------------------
void DriverNode::tick()
{
  const auto now = this->now();
  const auto tick_time = std::chrono::steady_clock::now();
  const double dt = std::clamp(
    std::chrono::duration<double>(tick_time - last_tick_).count(), 0.0, 0.1);
  last_tick_ = tick_time;
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

    std::optional<std::map<std::size_t, double>> joint_goals;
    const Limits * limits = &smooth_limits_;
    if (current_ && current_->started && !current_->done) {
      limits = &safety_limits_;
      const double t = seconds_since(current_->t0);
      if (current_->cancelled || current_->preempted || t >= current_->motion->duration()) {
        if (current_->joint_space) {
          // back to Cartesian targets consistent with where the joints are now (the head
          // pose only if the FK estimate is solvable, otherwise the previous target stays)
          target_antennas_ = {positions[7], positions[8]};
          target_body_yaw_ = positions[0];
          if (kinematics_.ik(head_pose_, target_body_yaw_)) {
            target_pose_ = head_pose_;
          }
        }
        current_->done = true;
      } else {
        const Targets targets =
          current_->motion->evaluate(std::min(t, current_->motion->duration() - 1e-3));
        if (targets.head) {
          target_pose_ = *targets.head;
        }
        if (targets.antennas) {
          target_antennas_ = *targets.antennas;
        }
        if (targets.body_yaw) {
          target_body_yaw_ = *targets.body_yaw;
        }
        joint_goals = targets.joints;
      }
    } else if (goto_) {
      limits = &safety_limits_;   // planned within the smooth limits already
      const double a = std::min(1.0, seconds_since(goto_->t0) / goto_->duration);
      const double s = ease(a);
      target_pose_ = interpolate(goto_->start_pose, goto_->end_pose, s);
      for (int i = 0; i < 2; ++i) {
        target_antennas_[i] = goto_->start_ant[i] + (goto_->end_ant[i] - goto_->start_ant[i]) * s;
      }
      target_body_yaw_ = goto_->start_yaw + (goto_->end_yaw - goto_->start_yaw) * s;
      if (a >= 1.0) {
        goto_.reset();
      }
    } else if (gaze_) {
      applyGaze();
    }

    if (joint_goals) {
      // written as is (the trajectory sets the speed), but the antennas and the body
      // yaw within their safety caps
      Positions g = last_goals_;
      for (const auto & [index, value] : *joint_goals) {
        double cap = std::numeric_limits<double>::infinity();
        if (index == 0) {
          cap = safety_limits_.body_yaw.velocity * dt;
        } else if (index == 7 || index == 8) {
          cap = safety_limits_.antennas.velocity * dt;
        }
        g[index] += std::clamp(value - g[index], -cap, cap);
      }
      goals = g;
      limiter_stale_ = true;
    } else {
      if (limiter_stale_) {
        limiter_.reset(targets());
        limiter_stale_ = false;
      }
      goals = computeGoals(limiter_.step(targets(), dt, *limits));
    }
    if (goals) {
      last_goals_ = *goals;
    }
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

std::optional<Positions> DriverNode::computeGoals(const Command & command)
{
  const auto joints = kinematics_.ik(command.head, command.body_yaw);
  if (!joints) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "target head pose is not reachable");
    return std::nullopt;
  }
  Positions goals;
  std::copy(joints->begin(), joints->end(), goals.begin());
  goals[7] = command.antennas[0];
  goals[8] = command.antennas[1];
  return goals;
}

// Called under mutex_: keep the head pointed at the gaze target (look_at).
void DriverNode::applyGaze()
{
  if (gaze_->glancing && std::chrono::steady_clock::now() >= gaze_->glance_end) {
    if (gaze_->previous) {
      gaze_ = Gaze{*gaze_->previous, false, {}, std::nullopt};
    } else {
      gaze_.reset();
      startGoto(kInitHeadPose, target_antennas_, target_body_yaw_, 1.0);
      return;
    }
  }
  geometry_msgs::msg::PointStamped in_base;
  try {
    auto target = gaze_->target;
    target.header.stamp = rclcpp::Time(0);   // latest available transform
    in_base = tf_buffer_->transform(target, base_frame_, tf2::durationFromSec(0.0));
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "look_at: cannot transform target from '%s': %s",
      gaze_->target.header.frame_id.c_str(), e.what());
    return;
  }
  target_pose_ = lookPose(Eigen::Vector3d(in_base.point.x, in_base.point.y, in_base.point.z));
}

Pose DriverNode::lookPose(const Eigen::Vector3d & point) const
{
  // the head frame at rest sits at the rest height above base_link, looking along +x
  const Eigen::Vector3d origin(0.0, 0.0, kinematics_.headZOffset());
  const Eigen::Vector3d d = point - origin;
  const double yaw = std::atan2(d.y(), d.x());
  const double pitch = std::clamp(-std::atan2(d.z(), std::hypot(d.x(), d.y())),
      -kMaxLookPitch, kMaxLookPitch);
  Pose p = Pose::Identity();
  p.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())).toRotationMatrix();
  return p;
}

// ---------------------------------------------------------------------------
// motions (replay_motion, execute_*_trajectory)
// ---------------------------------------------------------------------------
bool DriverNode::acceptMotionGoal(uint8_t priority)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (current_ && !current_->done && priority < current_->priority) {
    RCLCPP_WARN(
      get_logger(), "motion goal rejected: '%s' (priority %u) is running", current_->name.c_str(),
      current_->priority);
    return false;
  }
  return true;
}

std::shared_ptr<DriverNode::Execution> DriverNode::startMotion(
  std::shared_ptr<const Motion> motion, const std::string & name, uint8_t priority,
  double initial_goto)
{
  auto exec = std::make_shared<Execution>();
  exec->motion = motion;
  exec->name = name;
  exec->priority = priority;
  exec->joint_space = dynamic_cast<const JointMotion *>(motion.get()) != nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_ && !current_->done) {
      RCLCPP_INFO(get_logger(), "'%s' preempted by '%s'", current_->name.c_str(), name.c_str());
      current_->preempted = true;
      current_->done = true;
    }
    current_ = exec;          // reserves the slot: gaze and gotos yield to it
    goto_.reset();
  }
  // 1. reach the first sample smoothly (Cartesian motions only)
  const Targets first = motion->evaluate(0.0);
  if (initial_goto > 0.0 && first.head) {
    initial_goto = gotoPose(
      *first.head, first.antennas.value_or(target_antennas_), initial_goto, first.body_yaw);
    const auto until = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(initial_goto));
    while (std::chrono::steady_clock::now() < until && !exec->preempted && !exec->cancelled) {
      std::this_thread::sleep_for(10ms);
    }
  }
  // 2. play
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goto_.reset();
    exec->t0 = std::chrono::steady_clock::now();
    exec->started = true;
    if (exec->preempted || exec->cancelled) {
      exec->done = true;
    }
  }
  return exec;
}

template<typename Handle, typename Feedback>
uint8_t DriverNode::runMotion(
  std::shared_ptr<Execution> exec, std::shared_ptr<Handle> handle,
  std::shared_ptr<Feedback> feedback)
{
  while (!exec->done) {
    std::this_thread::sleep_for(50ms);
    if (handle->is_canceling()) {
      exec->cancelled = true;
      while (!exec->done) {
        std::this_thread::sleep_for(10ms);
      }
      return Result::ROS_ECANCELED;
    }
    const double elapsed = seconds_since(exec->t0);
    feedback->feedback.data_float =
      static_cast<float>(std::min(1.0, elapsed / exec->motion->duration()));
    feedback->feedback.data_str = exec->name;
    handle->publish_feedback(feedback);
  }
  if (exec->preempted) {
    return Result::ROS_EINTR;
  }
  if (exec->cancelled) {
    return Result::ROS_ECANCELED;
  }
  return Result::ROS_ENOERR;
}

std::vector<std::string> DriverNode::allMotionNames() const
{
  std::set<std::string> names;
  for (const auto & n : library_.names()) {
    names.insert(n);
  }
  for (const auto & ds : extra_datasets_) {
    try {
      for (const auto & n : library_.names(ds)) {
        names.insert(n);
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "cannot list dataset '%s': %s", ds.c_str(), e.what());
    }
  }
  return std::vector<std::string>(names.begin(), names.end());
}

void DriverNode::executeReplayMotion(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ReplayMotion>> handle)
{
  const auto goal = handle->get_goal();
  auto result = std::make_shared<ReplayMotion::Result>();
  std::shared_ptr<const RecordedMove> move;
  try {
    if (goal->name.empty()) {
      throw std::runtime_error("empty motion name");
    }
    try {
      move = library_.get(goal->name);
    } catch (const std::exception &) {
      for (const auto & ds : extra_datasets_) {
        try {
          move = library_.get(goal->name, ds);
          break;
        } catch (const std::exception &) {
        }
      }
      if (!move) {
        throw std::runtime_error("unknown motion '" + goal->name + "'");
      }
    }
  } catch (const std::exception & e) {
    // the spec asks for the list of available motions in that case
    result->result = makeResult(
      Result::ROS_EBADMSG, std::string(e.what()) + "; available: " + join(allMotionNames()));
    handle->abort(result);
    return;
  }
  RCLCPP_INFO(
    get_logger(), "playing '%s' (%.1f s) for %s", goal->name.c_str(), move->duration(),
    goal->meta.caller.empty() ? "unknown" : goal->meta.caller.c_str());
  auto exec = startMotion(
    std::make_shared<CartesianMotion>(move), goal->name, goal->meta.priority,
    motion_initial_goto_);
  auto feedback = std::make_shared<ReplayMotion::Feedback>();
  const uint8_t code = runMotion(exec, handle, feedback);
  result->result = makeResult(code);
  if (code == Result::ROS_ENOERR) {
    handle->succeed(result);
  } else if (code == Result::ROS_ECANCELED && handle->is_canceling()) {
    handle->canceled(result);   // otherwise cancelled by the driver itself (shutdown): aborted
  } else {
    handle->abort(result);
  }
}

void DriverNode::executeCartesianTrajectory(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteCartesianTrajectory>> handle)
{
  const auto goal = handle->get_goal();
  auto result = std::make_shared<ExecuteCartesianTrajectory::Result>();
  const auto & traj = goal->trajectory;
  std::vector<MoveSample> samples;
  std::string error;
  if (traj.points.empty()) {
    error = "empty trajectory";
  } else if (!traj.tracked_frame.empty() && traj.tracked_frame != "head") {
    error = "tracked_frame must be 'head' (got '" + traj.tracked_frame + "')";
  } else if (!traj.header.frame_id.empty() && traj.header.frame_id != base_frame_) {
    error = "poses must be expressed in '" + base_frame_ + "' (got '" + traj.header.frame_id + "')";
  } else {
    std::array<double, 2> antennas;
    double body_yaw;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      antennas = target_antennas_;
      body_yaw = target_body_yaw_;
    }
    double last = -1.0;
    for (const auto & p : traj.points) {
      const double t = rclcpp::Duration(p.time_from_start).seconds();
      if (t <= last) {
        error = "time_from_start must be strictly increasing";
        break;
      }
      last = t;
      // the trajectory is in base_frame; the IK works on rest-relative head poses
      samples.push_back({t, kinematics_.headFromBase(fromMsg(p.point.pose)), antennas, body_yaw});
    }
  }
  if (!error.empty()) {
    result->result = makeResult(Result::ROS_EBADMSG, error);
    handle->abort(result);
    return;
  }
  auto move = std::make_shared<const RecordedMove>(samples);
  RCLCPP_INFO(
    get_logger(), "execute_cartesian_trajectory: %zu points, %.1f s", samples.size(),
    move->duration());
  auto exec = startMotion(
    std::make_shared<CartesianMotion>(move), "cartesian_trajectory", goal->meta.priority,
    motion_initial_goto_);
  auto feedback = std::make_shared<ExecuteCartesianTrajectory::Feedback>();
  const uint8_t code = runMotion(exec, handle, feedback);
  result->result = makeResult(code);
  if (code == Result::ROS_ENOERR) {
    handle->succeed(result);
  } else if (code == Result::ROS_ECANCELED && handle->is_canceling()) {
    handle->canceled(result);   // otherwise cancelled by the driver itself (shutdown): aborted
  } else {
    handle->abort(result);
  }
}

void DriverNode::executeJointTrajectory(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteJointTrajectory>> handle)
{
  const auto goal = handle->get_goal();
  auto result = std::make_shared<ExecuteJointTrajectory::Result>();
  std::shared_ptr<JointMotion> motion;
  try {
    if (goal->safe_mode) {
      // the six platform joints are coupled: arbitrary combinations strain the mechanism
      for (const auto & name : goal->trajectory.joint_names) {
        if (name.rfind("stewart_", 0) == 0) {
          throw std::invalid_argument(
                  "joint '" + name + "' is only accepted with safe_mode=false (Stewart platform)");
        }
      }
    }
    std::vector<JointMotion::Point> points;
    for (const auto & p : goal->trajectory.points) {
      points.push_back({rclcpp::Duration(p.time_from_start).seconds(), p.positions});
    }
    if (!points.empty() && points.front().time > 0.0) {
      // start from the current goals rather than jumping to the first point
      JointMotion::Point start{0.0, {}};
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & name : goal->trajectory.joint_names) {
        const auto it = std::find(kMotorNames.begin(), kMotorNames.end(), name);
        start.positions.push_back(
          it == kMotorNames.end() ? 0.0 : last_goals_[it - kMotorNames.begin()]);
      }
      points.insert(points.begin(), start);
    }
    motion = std::make_shared<JointMotion>(goal->trajectory.joint_names, points);
  } catch (const std::exception & e) {
    result->result = makeResult(Result::ROS_EBADMSG, e.what());
    handle->abort(result);
    return;
  }
  RCLCPP_INFO(
    get_logger(), "execute_joint_trajectory: %zu joints, %.1f s",
    goal->trajectory.joint_names.size(), motion->duration());
  auto exec = startMotion(motion, "joint_trajectory", goal->meta.priority, 0.0);
  auto feedback = std::make_shared<ExecuteJointTrajectory::Feedback>();
  const uint8_t code = runMotion(exec, handle, feedback);
  result->result = makeResult(code);
  if (code == Result::ROS_ENOERR) {
    handle->succeed(result);
  } else if (code == Result::ROS_ECANCELED && handle->is_canceling()) {
    handle->canceled(result);   // otherwise cancelled by the driver itself (shutdown): aborted
  } else {
    handle->abort(result);
  }
}

// ---------------------------------------------------------------------------
// look_at
// ---------------------------------------------------------------------------
void DriverNode::executeLookAt(std::shared_ptr<rclcpp_action::ServerGoalHandle<LookAt>> handle)
{
  const auto goal = handle->get_goal();
  auto result = std::make_shared<LookAt::Result>();
  const std::string & policy = goal->policy;
  const bool has_target = !goal->target.header.frame_id.empty();

  if (policy == LookAt::Goal::RESET) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      gaze_.reset();
    }
    const double duration = gotoPose(kInitHeadPose, kInitAntennas, 1.0);
    std::this_thread::sleep_for(std::chrono::duration<double>(duration + 0.1));
    result->result = makeResult(Result::ROS_ENOERR);
    handle->succeed(result);
    return;
  }
  if (policy == LookAt::Goal::GLANCE || policy.empty()) {
    if (!has_target) {
      result->result = makeResult(Result::ROS_EBADMSG, "look_at: target.header.frame_id is empty");
      handle->abort(result);
      return;
    }
    if (policy.empty()) {
      // track until cancelled by a reset (or another target)
      std::lock_guard<std::mutex> lock(mutex_);
      gaze_ = Gaze{goal->target, false, {}, std::nullopt};
      result->result = makeResult(Result::ROS_ENOERR);
      handle->succeed(result);
      return;
    }
    // glance: look at the target for glance_duration, then back to what we were doing
    {
      std::lock_guard<std::mutex> lock(mutex_);
      std::optional<geometry_msgs::msg::PointStamped> previous;
      if (gaze_ && !gaze_->glancing) {
        previous = gaze_->target;
      }
      gaze_ = Gaze{goal->target, true,
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(glance_duration_)),
        previous};
    }
    const auto until = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(glance_duration_ + 1.0));
    while (std::chrono::steady_clock::now() < until) {
      std::this_thread::sleep_for(50ms);
      if (handle->is_canceling()) {
        std::lock_guard<std::mutex> lock(mutex_);
        gaze_.reset();
        result->result = makeResult(Result::ROS_ECANCELED);
        handle->canceled(result);
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      if (!gaze_ || !gaze_->glancing) {
        break;
      }
    }
    result->result = makeResult(Result::ROS_ENOERR);
    handle->succeed(result);
    return;
  }
  result->result = makeResult(
    Result::ROS_ENOTSUP, "look_at policy '" + policy + "' is not supported by the driver "
    "(supported: reset, glance, and target tracking with an empty policy)");
  handle->abort(result);
}

// ---------------------------------------------------------------------------
// list_motions
// ---------------------------------------------------------------------------
void DriverNode::listMotions(
  const std::shared_ptr<ListMotions::Request>/*request*/,
  std::shared_ptr<ListMotions::Response> response)
{
  try {
    response->motions = allMotionNames();
    response->result = makeResult(Result::ROS_ENOERR);
  } catch (const std::exception & e) {
    response->result = makeResult(Result::ROS_EOTHER, e.what());
  }
}

// ---------------------------------------------------------------------------
void DriverNode::shutdown()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_) {
      current_->cancelled = true;
      current_->done = true;
    }
    gaze_.reset();
  }
  if (get_parameter("sleep_on_shutdown").as_bool()) {
    RCLCPP_INFO(get_logger(), "going to sleep");
    const double duration = gotoPose(kSleepHeadPose, kSleepAntennas, 2.0);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(duration + 0.5));
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
