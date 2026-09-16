# Copyright 2026 SocialMinds Lab, IIIA-CSIC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Launch the Reachy Mini bare-metal driver with the parameters of config/driver.yaml.

    ros2 launch reachy_mini driver.launch.py                    # C++ driver, real servos
    ros2 launch reachy_mini driver.launch.py lang:=python       # Python driver
    ros2 launch reachy_mini driver.launch.py serial_port:=fake  # no hardware
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description."""
    params = os.path.join(get_package_share_directory('reachy_mini'), 'config', 'driver.yaml')
    lang = LaunchConfiguration('lang')
    return LaunchDescription([
        DeclareLaunchArgument('lang', default_value='cpp', choices=['cpp', 'python'],
                              description='which implementation of the driver to run'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyAMA3',
                              description="servo bus device, or 'fake' for a simulated bus"),
        DeclareLaunchArgument('params_file', default_value=params,
                              description='parameter file (see config/driver.yaml)'),
        Node(
            package=PythonExpression(["'reachy_mini_driver' if '", lang,
                                      "' == 'python' else 'reachy_mini_driver_cpp'"]),
            executable='driver_node',
            name='reachy_mini_driver',
            output='screen',
            parameters=[LaunchConfiguration('params_file'),
                        {'serial_port': LaunchConfiguration('serial_port')}],
        ),
    ])
