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
Publish the robot description and the fixed/revolute transforms of the Reachy Mini.

robot_state_publisher reads joint_states (body_yaw, stewart_1..6, right_antenna,
left_antenna) from the driver; the floating links (head, rod_1..6) are
broadcast by the driver itself. Add rviz:=true to open RViz (rviz/reachy_mini.rviz:
the model, TF and the camera image).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description."""
    share = get_package_share_directory('reachy_mini_description')
    with open(os.path.join(share, 'urdf', 'reachy_mini.urdf'), encoding='utf-8') as f:
        robot_description = f.read()
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false', description='open RViz'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(share, 'rviz', 'reachy_mini.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
