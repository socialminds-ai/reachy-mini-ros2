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
Launch the whole Reachy Mini.

The servo driver (parameters of config/driver.yaml), the robot model, the
camera and the audio (microphones and speaker).

    ros2 launch reachy_mini driver.launch.py                    # everything, C++ driver
    ros2 launch reachy_mini driver.launch.py lang:=python       # Python driver
    ros2 launch reachy_mini driver.launch.py serial_port:=fake  # no servos
    ros2 launch reachy_mini driver.launch.py camera:=false      # without the camera
    ros2 launch reachy_mini driver.launch.py audio:=false       # without microphones and speaker
    ros2 launch reachy_mini driver.launch.py asr:=false         # without speech recognition
    ros2 launch reachy_mini driver.launch.py head_taps:=false   # no waking up by tapping

The camera is reachy_mini_camera_driver (namespace /camera); the audio is
mic_frontend (/audio/head/raw) and pw_audio_play (plays /audio/out), both
through the robot's PipeWire and configured by config/mic_frontend/ and
config/pw_audio_play/, which launch_pal merges over their defaults. Speech
recognition is asr_whistle, on the head microphone (/humans/voices/head/speech),
configured by config/asr_whistle/; it needs the audio, and pauses while the
robot is asleep.

Also starts robot_blackboard, the robot-wide parameters (config/robot_blackboard/):
the driver starts asleep (sleep pose, then an antenna wiggle that says it is
ready) and writes /robot_blackboard/awake when /skill/wake_up or
/skill/go_to_sleep is called. reachy_mini_head_taps wakes the robot up when its
head is tapped 3 times, and puts it to sleep on 4 taps; it needs the audio.

Also starts robot_state_publisher with the reachy_mini_description model
(description:=false to skip it): joint_states from the driver give the body yaw,
the motor horns and the antennas; the driver itself broadcasts the head and the
six rods on /tf.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
        DeclareLaunchArgument('description', default_value='true',
                              description='publish the robot model (robot_state_publisher)'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='start the camera driver (reachy_mini_camera_driver)'),
        DeclareLaunchArgument('audio', default_value='true',
                              description='start the microphones (mic_frontend) and the '
                                          'speaker (pw_audio_play)'),
        DeclareLaunchArgument('asr', default_value='true',
                              description='start speech recognition (asr_whistle) on the '
                                          'head microphone; needs audio:=true'),
        DeclareLaunchArgument('head_taps', default_value='true',
                              description='wake up / go to sleep on 3 / 4 taps on the head '
                                          '(reachy_mini_head_taps); needs audio:=true'),
        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('robot_blackboard'),
                         'launch', 'robot_blackboard.launch.py'),
        ),
        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('reachy_mini_description'),
                         'launch', 'description.launch.py'),
            condition=IfCondition(LaunchConfiguration('description')),
        ),
        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('reachy_mini_camera_driver'),
                         'launch', 'camera.launch.py'),
            condition=IfCondition(LaunchConfiguration('camera')),
        ),
        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('mic_frontend'),
                         'launch', 'mic_frontend.launch.py'),
            condition=IfCondition(LaunchConfiguration('audio')),
        ),
        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('pw_audio_play'),
                         'launch', 'pw_audio_play.launch.py'),
            condition=IfCondition(LaunchConfiguration('audio')),
        ),
        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('asr_whistle'),
                         'launch', 'asr_whistle.launch.py'),
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('audio'), "'.lower() in ('true', '1') and '",
                 LaunchConfiguration('asr'), "'.lower() in ('true', '1')"])),
        ),
        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('reachy_mini_head_taps'),
                         'launch', 'head_taps.launch.py'),
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('audio'), "'.lower() in ('true', '1') and '",
                 LaunchConfiguration('head_taps'), "'.lower() in ('true', '1')"])),
        ),
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
