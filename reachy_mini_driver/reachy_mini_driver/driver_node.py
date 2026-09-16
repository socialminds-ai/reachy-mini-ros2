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
The driver node: a 50 Hz control loop around the servo bus.

Every tick:
  1. read the 9 motor positions, publish them as sensor_msgs/JointState
     (plus the head pose from forward kinematics);
  2. decide the target: the move being played, or the last head/antennas/body
     targets received on the topics, or the transition (goto) in progress;
  3. inverse kinematics -> 9 goal positions -> write them.

Everything that touches the hardware goes through `MotorBus`, so the node
runs unchanged on a `FakeBus` (tests, laptops).
"""

import signal
import threading
import time
from typing import List, Optional, Tuple

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from reachy_mini.kinematics.analytical_kinematics import AnalyticalKinematics
from reachy_mini.utils.interpolation import linear_pose_interpolation
from reachy_mini_msgs.action import PlayMove
from reachy_mini_msgs.msg import AntennasTarget
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from .bus import FakeBus, MOTOR_NAMES, MotorBus, RealBus
from .moves import DEFAULT_DATASET, MoveLibrary

# Same rest poses as the Pollen daemon.
INIT_HEAD_POSE = np.eye(4)
INIT_ANTENNAS = [-0.1745, 0.1745]           # ~10 deg outwards, less shaking than vertical
SLEEP_HEAD_POSE = np.array([
    [0.911, 0.004, 0.413, -0.021],
    [-0.004, 1.0, -0.001, 0.001],
    [-0.413, -0.001, 0.911, -0.044],
    [0.0, 0.0, 0.0, 1.0],
])
SLEEP_ANTENNAS = [-3.05, 3.05]


def pose_to_msg(pose: np.ndarray, stamp, frame_id: str) -> PoseStamped:
    """Convert a 4x4 matrix to a PoseStamped."""
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pose[:3, 3]
    q = R.from_matrix(pose[:3, :3]).as_quat()  # x, y, z, w
    msg.pose.orientation.x, msg.pose.orientation.y = q[0], q[1]
    msg.pose.orientation.z, msg.pose.orientation.w = q[2], q[3]
    return msg


def msg_to_pose(msg: PoseStamped) -> np.ndarray:
    """Convert a PoseStamped to a 4x4 matrix."""
    pose = np.eye(4)
    o = msg.pose.orientation
    pose[:3, :3] = R.from_quat([o.x, o.y, o.z, o.w]).as_matrix()
    pose[:3, 3] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    return pose


class Goto:
    """A timed, linear transition of head pose and antennas."""

    def __init__(self, start_pose, end_pose, start_ant, end_ant, duration: float) -> None:
        """Interpolate from start to end over `duration` seconds."""
        self.start_pose, self.end_pose = start_pose, end_pose
        self.start_ant, self.end_ant = list(start_ant), list(end_ant)
        self.duration = max(duration, 1e-3)
        self.t0 = time.monotonic()

    def evaluate(self) -> Tuple[np.ndarray, List[float], bool]:
        """Return (head pose, antennas, done)."""
        a = min(1.0, (time.monotonic() - self.t0) / self.duration)
        pose = linear_pose_interpolation(self.start_pose, self.end_pose, a)
        ant = [s + (e - s) * a for s, e in zip(self.start_ant, self.end_ant)]
        return pose, ant, a >= 1.0


class DriverNode(Node):
    """
    Reachy Mini hardware driver.

    ROS parameters
    --------------
    serial_port
        Servo bus device (default /dev/ttyAMA3). 'fake' uses an ideal simulated bus.
    rate
        Control loop frequency in Hz (default 50).
    dataset
        Default move library.
    wake_up_on_start, sleep_on_shutdown
        Go to the rest pose at start / to the sleep pose (and torque off) at exit.

    Topics
    ------
    joint_states (sensor_msgs/JointState), ~/head_pose (geometry_msgs/PoseStamped): published.
    ~/target_head_pose (PoseStamped), ~/target_antennas (reachy_mini_msgs/AntennasTarget),
    ~/target_body_yaw (std_msgs/Float64): subscribed.

    Action
    ------
    ~/play_move (reachy_mini_msgs/PlayMove).

    """

    def __init__(self, bus: Optional[MotorBus] = None) -> None:
        """Open the bus, start the loop."""
        super().__init__('reachy_mini_driver')
        self.declare_parameter('serial_port', '/dev/ttyAMA3')
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('dataset', DEFAULT_DATASET)
        self.declare_parameter('wake_up_on_start', True)
        self.declare_parameter('sleep_on_shutdown', True)

        port = self.get_parameter('serial_port').value
        self.bus = bus if bus is not None else (FakeBus() if port == 'fake' else RealBus(port))
        self.kinematics = AnalyticalKinematics(automatic_body_yaw=True)
        self.library = MoveLibrary(self.get_parameter('dataset').value)

        self._lock = threading.Lock()
        self._positions = self.bus.read_positions()
        self._head_pose = self.kinematics.fk(np.array(self._positions[:7]), no_iterations=20)
        self._target_pose = self._head_pose.copy()
        self._target_antennas = list(self._positions[7:9])
        self._target_body_yaw = float(self._positions[0])
        self._goto: Optional[Goto] = None
        self._move = None            # (RecordedMove, t0) while playing
        self._move_cancelled = False
        self._move_done = threading.Event()

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '~/head_pose', 10)
        self.create_subscription(PoseStamped, '~/target_head_pose', self._on_target_pose, 10)
        self.create_subscription(AntennasTarget, '~/target_antennas', self._on_target_antennas, 10)
        self.create_subscription(Float64, '~/target_body_yaw', self._on_target_body_yaw, 10)

        self._cb_group = ReentrantCallbackGroup()
        self._action = ActionServer(
            self, PlayMove, '~/play_move',
            execute_callback=self._execute_play_move,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._cb_group)

        self.bus.set_torque(True)
        if self.get_parameter('wake_up_on_start').value:
            self.goto(INIT_HEAD_POSE, INIT_ANTENNAS, duration=2.0)

        period = 1.0 / self.get_parameter('rate').value
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(f'driver ready on {port} at {1 / period:.0f} Hz')

    # ------------------------------------------------------------------ targets
    def goto(self, head_pose, antennas, duration: float) -> None:
        """Smoothly move to a head pose and antenna positions."""
        with self._lock:
            self._goto = Goto(self._target_pose, np.array(head_pose),
                              self._target_antennas, antennas, duration)

    def _on_target_pose(self, msg: PoseStamped) -> None:
        with self._lock:
            self._goto = None
            self._target_pose = msg_to_pose(msg)

    def _on_target_antennas(self, msg: AntennasTarget) -> None:
        with self._lock:
            self._target_antennas = [msg.right, msg.left]

    def _on_target_body_yaw(self, msg: Float64) -> None:
        with self._lock:
            self._target_body_yaw = msg.data

    # --------------------------------------------------------------- the loop
    def _tick(self) -> None:
        now = self.get_clock().now().to_msg()
        try:
            positions = self.bus.read_positions()
        except Exception as e:  # noqa: BLE001 - a bus hiccup must not kill the loop
            self.get_logger().warning(f'read failed: {e}', throttle_duration_sec=1.0)
            return

        with self._lock:
            self._positions = positions
            self._head_pose = self.kinematics.fk(np.array(positions[:7]), no_iterations=3)

            if self._move is not None:
                move, t0 = self._move
                t = time.monotonic() - t0
                if self._move_cancelled or t >= move.duration:
                    self._move = None
                    self._move_done.set()
                else:
                    head, antennas, body_yaw = move.evaluate(min(t, move.duration - 1e-2))
                    if head is not None:
                        self._target_pose = np.array(head)
                    if antennas is not None:
                        self._target_antennas = list(antennas)
                    if body_yaw is not None:
                        self._target_body_yaw = float(body_yaw)
            elif self._goto is not None:
                pose, ant, done = self._goto.evaluate()
                self._target_pose, self._target_antennas = pose, ant
                if done:
                    self._goto = None

            goals = self._compute_goals()

        js = JointState()
        js.header.stamp = now
        js.name = MOTOR_NAMES
        js.position = positions
        self.joint_pub.publish(js)
        self.pose_pub.publish(pose_to_msg(self._head_pose, now, 'reachy_mini_base'))

        if goals is not None:
            try:
                self.bus.write_goal_positions(goals)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warning(f'write failed: {e}', throttle_duration_sec=1.0)

    def _compute_goals(self) -> Optional[List[float]]:
        """Inverse kinematics of the current target; None if unreachable."""
        joints = self.kinematics.ik(self._target_pose, body_yaw=self._target_body_yaw)
        if joints is None or np.any(np.isnan(joints)):
            self.get_logger().warning('target head pose is not reachable',
                                      throttle_duration_sec=1.0)
            return None
        return [float(j) for j in joints] + [float(a) for a in self._target_antennas]

    # ------------------------------------------------------------- play_move
    def _execute_play_move(self, goal_handle):
        goal = goal_handle.request
        result = PlayMove.Result()
        try:
            move = self.library.get(goal.move, goal.dataset or None)
        except Exception as e:  # noqa: BLE001 - report loading errors to the caller
            result.message = str(e)
            goal_handle.abort()
            return result

        self.get_logger().info(f"playing '{goal.move}' ({move.duration:.1f} s)")
        # 1. reach the first pose of the move (so the move does not start with a jump)
        head0, ant0, _ = move.evaluate(0.0)
        if goal.initial_goto_duration > 0:
            self.goto(head0, list(ant0), goal.initial_goto_duration)
            time.sleep(goal.initial_goto_duration)
        # 2. play, ticking feedback until the loop says it is over
        with self._lock:
            self._move_cancelled = False
            self._move_done.clear()
            self._move = (move, time.monotonic())
        feedback = PlayMove.Feedback()
        while not self._move_done.wait(timeout=0.1):
            if goal_handle.is_cancel_requested:
                with self._lock:
                    self._move_cancelled = True
                self._move_done.wait(timeout=1.0)
                goal_handle.canceled()
                result.message = 'cancelled'
                return result
            with self._lock:
                elapsed = time.monotonic() - self._move[1] if self._move else move.duration
            feedback.elapsed = float(elapsed)
            feedback.progress = float(min(1.0, elapsed / move.duration))
            goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        result.completed = True
        result.message = 'done'
        return result

    def stop_move(self) -> None:
        """Interrupt the move being played (used at shutdown)."""
        with self._lock:
            self._move_cancelled = True

    # -------------------------------------------------------------- shutdown
    def shutdown(self) -> None:
        """Go to sleep and release the motors."""
        self.stop_move()
        self._timer.cancel()          # the executor may already be gone: tick by hand
        if self.get_parameter('sleep_on_shutdown').value:
            self.get_logger().info('going to sleep')
            self.goto(SLEEP_HEAD_POSE, SLEEP_ANTENNAS, duration=2.0)
            deadline = time.monotonic() + 2.5
            while self._goto is not None and time.monotonic() < deadline:
                self._tick()
                time.sleep(0.02)
        self.bus.set_torque(False)
        self.bus.close()


def main(args=None) -> None:
    """
    Entry point.

    rclpy's default SIGINT handler tears the ROS context down immediately,
    which would make the sleep sequence fail to publish. We install our own
    handler: it only asks the loop to stop, so shutdown() runs with a valid
    context, then we shut ROS down ourselves.
    """
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DriverNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    try:
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        node.shutdown()
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
