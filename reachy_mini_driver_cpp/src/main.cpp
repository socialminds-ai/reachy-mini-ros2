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

#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "reachy_mini_driver_cpp/driver_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<reachy_mini_driver::DriverNode> node;
  try {
    node = std::make_shared<reachy_mini_driver::DriverNode>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("reachy_mini_driver"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  node->shutdown();
  rclcpp::shutdown();
  return 0;
}
