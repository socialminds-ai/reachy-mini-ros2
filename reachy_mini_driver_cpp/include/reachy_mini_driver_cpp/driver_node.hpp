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

// The driver node: a 50 Hz control loop around the servo bus, exposed through
// the ROS4HRI skills (https://ros4hri.github.io/skills/). Every tick:
//   1. read the 9 motor positions, publish JointState, head pose (FK) and TF;
//   2. decide the target: the motion being executed (replay_motion,
//      execute_cartesian_trajectory, execute_joint_trajectory), else the goto in
//      progress, else the look_at gaze target, else the last targets received on
//      the topics;
//   3. inverse kinematics -> 9 goal positions -> write them.
//
// Skills (paths under the `skills_namespace` parameter, default /skill):
//   replay_motion, list_motions, execute_cartesian_trajectory,
//   execute_joint_trajectory (motions_skills) and look_at (interaction_skills).
// Results use the std_skills error codes; a motion goal with a priority greater
// than or equal to the running one preempts it (ROS_EINTR), a lower one is rejected.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
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
#include "reachy_mini_driver_cpp/kinematics.hpp"
#include "reachy_mini_driver_cpp/limits.hpp"
#include "reachy_mini_driver_cpp/moves.hpp"
#include "reachy_mini_driver_cpp/trajectories.hpp"
#include "reachy_mini_msgs/msg/antennas_target.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_skills/msg/result.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace reachy_mini_driver
{

class DriverNode : public rclcpp::Node
{
public:
  using ReplayMotion = motions_skills::action::ReplayMotion;
  using ExecuteCartesianTrajectory = motions_skills::action::ExecuteCartesianTrajectory;
  using ExecuteJointTrajectory = motions_skills::action::ExecuteJointTrajectory;
  using LookAt = interaction_skills::action::LookAt;
  using ListMotions = motions_skills::srv::ListMotions;
  using Result = std_skills::msg::Result;

  /// `bus` overrides the serial_port parameter (tests).
  explicit DriverNode(
    std::shared_ptr<MotorBus> bus = nullptr,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DriverNode() override;

  /// Smoothly move to a head pose, antenna positions and (if given) body yaw, in
  /// `duration` seconds or longer if the smooth speed limits need it; returns the
  /// duration actually used.
  double gotoPose(
    const Pose & head, const std::array<double, 2> & antennas, double duration,
    std::optional<double> body_yaw = std::nullopt);
  /// Go to the sleep pose and release the motors (call before destroying).
  void shutdown();

  std::shared_ptr<MotorBus> bus() {return bus_;}

private:
  struct Goto
  {
    Pose start_pose, end_pose;
    std::array<double, 2> start_ant, end_ant;
    double start_yaw, end_yaw;
    double duration;
    std::chrono::steady_clock::time_point t0;
  };

  /// One motion being executed; the single "motion slot" of the driver.
  struct Execution
  {
    std::shared_ptr<const Motion> motion;
    std::string name;
    uint8_t priority{0};
    std::chrono::steady_clock::time_point t0;
    bool joint_space{false};
    std::atomic<bool> started{false};   ///< the initial goto is over, samples are applied
    std::atomic<bool> done{false};
    std::atomic<bool> cancelled{false};
    std::atomic<bool> preempted{false};
  };

  /// A look_at target: either a point fixed in base_frame_ (resolved once, at goal reception),
  /// or, when tracking, a point in its own frame, re-resolved at every tick.
  struct GazeTarget
  {
    geometry_msgs::msg::PointStamped point;
    bool tracking{false};
    uint64_t id{0};                                  ///< identifies the tracking goal
    std::chrono::steady_clock::time_point last_seen;   ///< last successful resolution (tracking)
    bool lost{false};                                ///< not resolved for tracking_lost_timeout
  };

  /// look_at state: a point to keep the head pointed at, possibly for a limited time (glance).
  struct Gaze
  {
    GazeTarget target;
    bool glancing{false};
    std::chrono::steady_clock::time_point glance_end;
    std::optional<GazeTarget> previous;   // restored after a glance
  };

  void tick();
  std::optional<Positions> computeGoals(const Command & command);
  /// Start a goto from the current targets (call under mutex_); returns its duration.
  double startGoto(
    const Pose & head, const std::array<double, 2> & antennas, double body_yaw,
    double duration);
  Command targets() const {return Command{target_pose_, target_antennas_, target_body_yaw_};}
  void applyGaze();
  /// Express a look_at target in base_frame_ (latest TF); nullopt (and `error`) on failure.
  std::optional<geometry_msgs::msg::PointStamped> resolveGazeTarget(
    const geometry_msgs::msg::PointStamped & target, double timeout, std::string & error) const;
  /// Start `motion` in the motion slot (preempting the current one); returns the execution.
  std::shared_ptr<Execution> startMotion(
    std::shared_ptr<const Motion> motion, const std::string & name, uint8_t priority,
    double initial_goto);
  /// Wait for `exec` to finish, publishing `feedback(progress)`; returns the std_skills error code.
  template<typename Handle, typename Feedback>
  uint8_t runMotion(
    std::shared_ptr<Execution> exec, std::shared_ptr<Handle> handle,
    std::shared_ptr<Feedback> feedback);
  /// Head pose that points the head towards `point` (in the base frame).
  Pose lookPose(const Eigen::Vector3d & point) const;
  bool acceptMotionGoal(uint8_t priority);

  void executeReplayMotion(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ReplayMotion>> handle);
  void executeCartesianTrajectory(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteCartesianTrajectory>> handle);
  void executeJointTrajectory(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteJointTrajectory>> handle);
  /// Run a goal's execution in its own thread, counted in goal_threads_; its exceptions
  /// are caught (after a Ctrl+C, the goal handle throws when publishing).
  void runGoal(const std::string & skill, std::function<void()> execute);
  /// Wait (bounded) for the goal threads to end; call once their motion/gaze is cancelled.
  void waitForGoalThreads();
  void executeLookAt(std::shared_ptr<rclcpp_action::ServerGoalHandle<LookAt>> handle);
  /// Warn loudly if `frame` is a robot frame that moves with the gaze; call under mutex_.
  void checkTrackedFrame(const std::string & frame);
  void executeTracking(std::shared_ptr<rclcpp_action::ServerGoalHandle<LookAt>> handle);
  void listMotions(
    const std::shared_ptr<ListMotions::Request> request,
    std::shared_ptr<ListMotions::Response> response);
  std::vector<std::string> allMotionNames() const;

  std::shared_ptr<MotorBus> bus_;
  Kinematics kinematics_;
  MoveLibrary library_;
  std::vector<std::string> extra_datasets_;

  std::mutex mutex_;
  Positions positions_{};
  Positions last_goals_{};
  Pose head_pose_{Pose::Identity()};
  Pose target_pose_{Pose::Identity()};
  std::array<double, 2> target_antennas_{};
  double target_body_yaw_{0.0};
  std::optional<Goto> goto_;
  std::shared_ptr<Execution> current_;
  std::optional<Gaze> gaze_;
  double glance_duration_{2.0};
  double tracking_lost_timeout_{1.0};
  uint64_t gaze_ids_{0};
  std::atomic<int> goal_threads_{0};
  /// The robot's frames (URDF links from /robot_description, plus the ones the driver
  /// publishes): they all move with the gaze (head, or body through the automatic body
  /// yaw), except base_frame_.
  std::set<std::string> robot_frames_;
  double motion_initial_goto_{0.5};
  /// The commanded pose follows the targets within `smooth_limits_` (gaze, topics,
  /// gotos are planned within them) or, while a motion plays, `safety_limits_`.
  MotionLimiter limiter_;
  bool limiter_stale_{false};   ///< joint goals were written directly: resync first
  Limits smooth_limits_{};
  Limits safety_limits_{};
  std::chrono::steady_clock::time_point last_tick_{};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string base_frame_{"base_link"};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<reachy_mini_msgs::msg::AntennasTarget>::SharedPtr antennas_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr body_yaw_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
  rclcpp_action::Server<ReplayMotion>::SharedPtr replay_server_;
  rclcpp_action::Server<ExecuteCartesianTrajectory>::SharedPtr cartesian_server_;
  rclcpp_action::Server<ExecuteJointTrajectory>::SharedPtr joint_server_;
  rclcpp_action::Server<LookAt>::SharedPtr look_at_server_;
  rclcpp::Service<ListMotions>::SharedPtr list_service_;
};

extern const Pose kInitHeadPose;
extern const Pose kSleepHeadPose;
extern const std::array<double, 2> kInitAntennas;
extern const std::array<double, 2> kSleepAntennas;

}  // namespace reachy_mini_driver

#endif  // REACHY_MINI_DRIVER_CPP__DRIVER_NODE_HPP_
