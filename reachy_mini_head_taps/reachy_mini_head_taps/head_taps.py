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
Wake the robot up, or put it to sleep, by tapping on its head.

Listens to the head microphones; `wake_taps` taps (3) call the driver's
`wake_up` skill (with the `wake_motion` animation, Pollen's `wake-mini-up`, the
first time the robot wakes up; `rewake_motion`, none by default, after that),
`sleep_taps` taps (4) its `go_to_sleep` skill. The robot's state is followed on
the robot blackboard (`awake_parameter`, /robot_blackboard/awake, written by
the driver): tapping to wake up a robot that is already awake, or to put to
sleep a robot already asleep, does nothing. Taps are ignored while the robot
moves: the motors would be heard as taps.
"""

from audio_common_msgs.msg import AudioData, AudioInfo
from motions_skills.action import ReplayMotion
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from reachy_mini_head_taps.taps import TapCounter, TapDetector, to_float
from std_msgs.msg import Int32
from std_skills.action import Empty as EmptySkill
from std_skills.msg import Meta, Result


class HeadTaps(Node):

    def __init__(self):
        super().__init__('head_taps')
        p = self.declare_parameter
        self.audio_topic = p('audio_topic', '/audio/head/raw').value
        self.wake_taps = p('wake_taps', 3).value
        self.sleep_taps = p('sleep_taps', 4).value
        self.wake_motion = p('wake_motion', 'wake-mini-up').value
        self.rewake_motion = p('rewake_motion', '').value
        self.skills_ns = p('skills_namespace', '/skill').value
        awake_parameter = p('awake_parameter', '/robot_blackboard/awake').value
        self.holdoff = p('holdoff', 1.0).value      # deaf for this long after a motion
        self.detector_params = {
            'onset_ratio_db': p('onset_ratio_db', 20.0).value,
            'min_level_dbfs': p('min_level_dbfs', -20.0).value,
            'decay_db': p('decay_db', 20.0).value,
            'refractory_s': p('refractory', 0.1).value,
        }
        self.counter = TapCounter(p('max_gap', 0.7).value)

        self.format = ('S16LE', 1, 16000)      # mic_frontend's, until the info says otherwise
        self.detector = None
        self.audio_time = 0.0
        self.busy = False
        self.deaf_until = 0.0

        self.wake_up_client = ActionClient(self, ReplayMotion, self.skills_ns + '/wake_up')
        self.sleep_client = ActionClient(self, EmptySkill, self.skills_ns + '/go_to_sleep')
        self.taps_pub = self.create_publisher(Int32, '~/taps', 10)

        # the robot's state, from the blackboard: None while unknown
        self.awake = None
        self.has_woken_up = False   # since we started, by us or anyone else
        self.blackboard = None
        if awake_parameter:
            node, _, self.awake_name = awake_parameter.rpartition('/')
            if not node.startswith('/') or not self.awake_name:
                raise ValueError(f"awake_parameter '{awake_parameter}' is not of the form "
                                 '/<node>/<parameter>')
            self.blackboard_node = node
            self.blackboard = AsyncParameterClient(self, node)
            self.blackboard.on_parameter_event(self.on_parameter_event)
            self.blackboard_ready = False
            # read it when the blackboard appears (it may start after us, or restart)
            self.create_timer(1.0, self.poll_blackboard)

        info_topic = self.audio_topic.rsplit('/', 1)[0] + '/info'
        self.create_subscription(AudioInfo, info_topic, self.on_info, QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            AudioData, self.audio_topic, self.on_audio, qos_profile_sensor_data)
        self.get_logger().info(
            f'listening on {self.audio_topic}: {self.wake_taps} taps to wake up, '
            f'{self.sleep_taps} taps to sleep')

    # --- the robot's state -----------------------------------------------------

    def set_awake(self, value):
        if not isinstance(value, bool):
            self.get_logger().warn(f'{self.blackboard_node}/{self.awake_name} is not a bool: '
                                   f'{value!r}')
            value = None
        if value != self.awake:
            self.get_logger().info(
                {True: 'the robot is awake', False: 'the robot is asleep',
                 None: 'the robot state is unknown'}[value])
        self.awake = value
        self.has_woken_up |= value is True

    def poll_blackboard(self):
        ready = self.blackboard.services_are_ready()
        appeared = ready and not self.blackboard_ready
        self.blackboard_ready = ready
        if appeared:
            self.blackboard.get_parameters([self.awake_name]).add_done_callback(
                lambda f: self.set_awake(
                    rclpy.parameter.parameter_value_to_python(f.result().values[0])
                    if f.result() and f.result().values else None))

    def on_parameter_event(self, event):
        if event.node != self.blackboard_node:
            return
        for p in list(event.new_parameters) + list(event.changed_parameters):
            if p.name == self.awake_name:
                self.set_awake(rclpy.parameter.parameter_value_to_python(p.value))
        if any(p.name == self.awake_name for p in event.deleted_parameters):
            self.set_awake(None)

    # --- taps ----------------------------------------------------------------

    def on_info(self, msg):
        fmt = (msg.sample_format, msg.channels, msg.sample_rate)
        if fmt != self.format:
            self.format = fmt
            self.detector = None

    def on_audio(self, msg):
        sample_format, channels, rate = self.format
        try:
            x = to_float(msg.data, sample_format, channels)
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return
        if self.detector is None:
            self.detector = TapDetector(rate, **self.detector_params)
            self.audio_time = 0.0
        taps = self.detector.process(x)
        self.audio_time += len(x) / rate
        if self.busy or self.audio_time < self.deaf_until:
            return
        for t in taps:
            self.get_logger().debug(f'tap at {t:.2f} s')
            self.counter.add(t)
        n = self.counter.poll(self.audio_time)
        if n:
            self.taps_pub.publish(Int32(data=n))
            self.get_logger().info(f'{n} taps')
            if n == self.wake_taps:
                self.wake_up()
            elif n == self.sleep_taps:
                self.go_to_sleep()

    # --- skills --------------------------------------------------------------

    def wake_up(self):
        if self.awake is True:
            self.get_logger().info('already awake')
            return
        # the long animation once, a short one (or none) afterwards
        motion = self.rewake_motion if self.has_woken_up else self.wake_motion
        self.has_woken_up = True     # also when the state is not followed
        self.send(self.wake_up_client, ReplayMotion.Goal(name=motion),
                  'waking up' + (f' ({motion})' if motion else ''))

    def go_to_sleep(self):
        if self.awake is False:
            self.get_logger().info('already asleep')
            return
        self.send(self.sleep_client, EmptySkill.Goal(), 'going to sleep')

    def send(self, client, goal, what):
        """Send a skill goal; deaf until it ends."""
        if not client.server_is_ready():
            self.get_logger().warn(f'{what}: the skill server is not available')
            return
        goal.meta.priority = Meta.NORMAL_PRIORITY
        goal.meta.caller = self.get_fully_qualified_name()
        self.get_logger().info(what)
        self.busy = True

        def done():
            self.busy = False
            self.deaf_until = self.audio_time + self.holdoff

        def on_result(future):
            result = future.result().result.result
            if result.error_code != Result.ROS_ENOERR:
                self.get_logger().warn(f'{what}: failed ({result.error_msg})')
            done()

        def on_goal(future):
            handle = future.result()
            if not handle.accepted:
                self.get_logger().warn(f'{what}: goal rejected')
                done()
                return
            handle.get_result_async().add_done_callback(on_result)

        client.send_goal_async(goal).add_done_callback(on_goal)


def main(args=None):
    rclpy.init(args=args)
    node = HeadTaps()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
