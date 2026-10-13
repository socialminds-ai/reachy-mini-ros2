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
"""Tests of the driver on the fake bus: control loop, targets, motion skills."""

import os
import threading
import time

from geometry_msgs.msg import PoseStamped
from motions_skills.action import ReplayMotion
from motions_skills.srv import ListMotions
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from reachy_mini_driver.bus import FakeBus
from reachy_mini_driver.driver_node import DriverNode, INIT_ANTENNAS, SLEEP_ANTENNAS
from sensor_msgs.msg import JointState
from std_skills.action import Empty as EmptySkill
from std_skills.msg import Result
from tf2_msgs.msg import TFMessage

MOVES_DIR = os.path.join(os.path.dirname(__file__), 'moves')


@pytest.fixture
def driver():
    # a private domain: colcon runs the packages' tests in parallel, and the C++ driver test
    # starts the same node name and skill servers
    os.environ.setdefault('ROS_DOMAIN_ID', '71')
    rclpy.init()
    bus = FakeBus()
    node = DriverNode(bus=bus, parameter_overrides=[
        rclpy.parameter.Parameter('dataset', value=MOVES_DIR)])
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield node, bus
    node.set_parameters([rclpy.parameter.Parameter('sleep_on_shutdown', value=False)])
    node.shutdown()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


def wait_for(predicate, timeout=5.0):
    """Poll `predicate` until true or timeout."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_torque_is_enabled_and_loop_writes(driver):
    node, bus = driver
    assert bus.torque
    assert wait_for(lambda: bus.writes > 10)


def test_wake_up_reaches_init_pose(driver):
    node, bus = driver
    # after the 2 s wake-up goto, antennas are at INIT and the head is upright
    assert wait_for(lambda: abs(bus.positions[7] - INIT_ANTENNAS[0]) < 1e-3
                    and abs(bus.positions[8] - INIT_ANTENNAS[1]) < 1e-3, timeout=4.0)
    assert abs(bus.positions[0]) < 1e-6          # body yaw 0


def test_joint_states_are_published(driver):
    node, _ = driver
    received = []
    listener = rclpy.create_node('listener')
    listener.create_subscription(JointState, '/joint_states', received.append, 10)
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and len(received) < 5:
        rclpy.spin_once(listener, timeout_sec=0.05)
    assert len(received) >= 5
    assert received[0].name[0] == 'body_yaw' and len(received[0].position) == 9
    listener.destroy_node()


def test_target_head_pose_moves_the_platform(driver):
    node, bus = driver
    time.sleep(2.2)                              # let the wake-up finish
    before = list(bus.positions[1:7])
    msg = PoseStamped()
    msg.pose.position.z = 0.02                   # 2 cm up
    msg.pose.orientation.w = 1.0
    node._on_target_pose(msg)
    assert wait_for(lambda: max(abs(a - b) for a, b in zip(bus.positions[1:7], before)) > 0.05)


def test_replay_motion_skill(driver):
    node, bus = driver
    time.sleep(2.2)
    client_node = rclpy.create_node('client')
    client = ActionClient(client_node, ReplayMotion, '/skill/replay_motion')
    assert client.wait_for_server(timeout_sec=5.0)
    goal = ReplayMotion.Goal(name='nod')
    goal.meta.caller = 'test'
    feedbacks = []
    future = client.send_goal_async(goal, feedback_callback=feedbacks.append)
    rclpy.spin_until_future_complete(client_node, future, timeout_sec=5.0)
    handle = future.result()
    assert handle.accepted
    result_future = handle.get_result_async()
    # the antennas must wiggle while the 1 s move plays
    assert wait_for(lambda: abs(bus.positions[7] + 0.3 * 1.0) < 0.1, timeout=2.0)
    rclpy.spin_until_future_complete(client_node, result_future, timeout_sec=5.0)
    result = result_future.result().result
    assert result.result.error_code == Result.ROS_ENOERR
    assert feedbacks and feedbacks[-1].feedback.feedback.data_float > 0.5
    client_node.destroy_node()


def test_replay_motion_unknown_lists_the_motions(driver):
    node, _ = driver
    client_node = rclpy.create_node('client2')
    client = ActionClient(client_node, ReplayMotion, '/skill/replay_motion')
    assert client.wait_for_server(timeout_sec=5.0)
    future = client.send_goal_async(ReplayMotion.Goal(name='does_not_exist'))
    rclpy.spin_until_future_complete(client_node, future, timeout_sec=5.0)
    result_future = future.result().get_result_async()
    rclpy.spin_until_future_complete(client_node, result_future, timeout_sec=5.0)
    result = result_future.result().result.result
    assert result.error_code == Result.ROS_EBADMSG
    assert 'nod' in result.error_msg
    client_node.destroy_node()


def run_goal(client_node, action_type, name, goal):
    """Send `goal` to the action `name` and wait for its std_skills result."""
    client = ActionClient(client_node, action_type, name)
    assert client.wait_for_server(timeout_sec=5.0)
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(client_node, future, timeout_sec=5.0)
    result_future = future.result().get_result_async()
    rclpy.spin_until_future_complete(client_node, result_future, timeout_sec=10.0)
    return result_future.result().result.result


def test_go_to_sleep_and_wake_up(driver):
    node, bus = driver
    time.sleep(2.2)
    client_node = rclpy.create_node('client4')

    def antennas_at(a):
        return abs(bus.positions[7] - a[0]) < 1e-2 and abs(bus.positions[8] - a[1]) < 1e-2

    sleep = EmptySkill.Goal()
    sleep.meta.caller = 'test'
    assert run_goal(client_node, EmptySkill, '/skill/go_to_sleep', sleep).error_code == \
        Result.ROS_ENOERR
    assert antennas_at(SLEEP_ANTENNAS)
    assert bus.torque                            # asleep, not released

    # default wake-up: a goto to the rest pose, torque back on if it was off
    bus.torque = False
    wake = ReplayMotion.Goal()
    assert run_goal(client_node, ReplayMotion, '/skill/wake_up', wake).error_code == \
        Result.ROS_ENOERR
    assert bus.torque
    assert antennas_at(INIT_ANTENNAS)
    assert abs(bus.positions[0]) < 1e-3

    # with an animation: played, then back to the rest pose
    wiggled = []
    checker = threading.Thread(target=lambda: wiggled.append(
        wait_for(lambda: abs(bus.positions[7] + 0.3) < 0.1, timeout=6.0)))
    checker.start()
    wake.name = 'nod'
    assert run_goal(client_node, ReplayMotion, '/skill/wake_up', wake).error_code == \
        Result.ROS_ENOERR
    checker.join()
    assert wiggled == [True]
    assert antennas_at(INIT_ANTENNAS)

    wake.name = 'does_not_exist'
    result = run_goal(client_node, ReplayMotion, '/skill/wake_up', wake)
    assert result.error_code == Result.ROS_EBADMSG and 'nod' in result.error_msg
    client_node.destroy_node()


def test_list_motions_skill(driver):
    node, _ = driver
    client_node = rclpy.create_node('client3')
    client = client_node.create_client(ListMotions, '/skill/list_motions')
    assert client.wait_for_service(timeout_sec=5.0)
    future = client.call_async(ListMotions.Request())
    rclpy.spin_until_future_complete(client_node, future, timeout_sec=5.0)
    assert future.result().motions == ['nod']
    assert future.result().result.error_code == Result.ROS_ENOERR
    client_node.destroy_node()


def test_tf_frames_are_broadcast(driver):
    node, _ = driver
    received = []
    listener = rclpy.create_node('tf_listener')
    listener.create_subscription(TFMessage, '/tf', received.append, 10)
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and len(received) < 3:
        rclpy.spin_once(listener, timeout_sec=0.05)
    assert received
    children = {t.child_frame_id for t in received[-1].transforms}
    assert children == {'head', 'rod_1', 'rod_2', 'rod_3', 'rod_4', 'rod_5', 'rod_6'}
    head = next(t for t in received[-1].transforms if t.child_frame_id == 'head')
    assert head.header.frame_id == 'base_link'
    assert 0.15 < head.transform.translation.z < 0.20      # rest height of the head
    listener.destroy_node()
