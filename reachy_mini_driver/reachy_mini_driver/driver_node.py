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
import sys
import threading
import time
from typing import List, Optional, Tuple

from geometry_msgs.msg import PoseStamped, TransformStamped
from motions_skills.action import ReplayMotion
from motions_skills.srv import ListMotions
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from reachy_mini.kinematics.analytical_kinematics import AnalyticalKinematics
from reachy_mini.utils.interpolation import linear_pose_interpolation
from reachy_mini_msgs.msg import AntennasTarget
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_skills.action import Empty as EmptySkill
from std_skills.msg import Result
from tf2_ros import TransformBroadcaster

from .bus import FakeBus, MOTOR_NAMES, MotorBus, RealBus
from .geometry import PlatformGeometry
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


def pose_to_transform(pose: np.ndarray, stamp, parent: str, child: str) -> TransformStamped:
    """Convert a 4x4 matrix to a TransformStamped parent -> child."""
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = pose[:3, 3]
    q = R.from_matrix(pose[:3, :3]).as_quat()
    t.transform.rotation.x, t.transform.rotation.y = q[0], q[1]
    t.transform.rotation.z, t.transform.rotation.w = q[2], q[3]
    return t


def msg_to_pose(msg: PoseStamped) -> np.ndarray:
    """Convert a PoseStamped to a 4x4 matrix."""
    pose = np.eye(4)
    o = msg.pose.orientation
    pose[:3, :3] = R.from_quat([o.x, o.y, o.z, o.w]).as_matrix()
    pose[:3, 3] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    return pose


class Goto:
    """A timed, linear transition of head pose, antennas and body yaw."""

    def __init__(self, start_pose, end_pose, start_ant, end_ant, start_yaw: float,
                 end_yaw: float, duration: float) -> None:
        """Interpolate from start to end over `duration` seconds."""
        self.start_pose, self.end_pose = start_pose, end_pose
        self.start_ant, self.end_ant = list(start_ant), list(end_ant)
        self.start_yaw, self.end_yaw = start_yaw, end_yaw
        self.duration = max(duration, 1e-3)
        self.t0 = time.monotonic()

    def evaluate(self) -> Tuple[np.ndarray, List[float], float, bool]:
        """Return (head pose, antennas, body yaw, done)."""
        a = min(1.0, (time.monotonic() - self.t0) / self.duration)
        pose = linear_pose_interpolation(self.start_pose, self.end_pose, a)
        ant = [s + (e - s) * a for s, e in zip(self.start_ant, self.end_ant)]
        yaw = self.start_yaw + (self.end_yaw - self.start_yaw) * a
        return pose, ant, yaw, a >= 1.0


class GotoMove:
    """A Goto as a move (same interface as RecordedMove), to run it in the motion slot."""

    def __init__(self, start_pose, end_pose, start_ant, end_ant, start_yaw: float,
                 end_yaw: float, duration: float) -> None:
        """Interpolate from start to end over `duration` seconds."""
        self.goto = Goto(start_pose, end_pose, start_ant, end_ant, start_yaw, end_yaw, duration)
        # held at the end for a few ticks: the loop stops evaluating a move just before its end
        self.duration = self.goto.duration + 0.1

    def evaluate(self, t: float):
        """Return (head pose, antennas, body yaw) at time t."""
        a = min(1.0, max(0.0, t / self.goto.duration))
        g = self.goto
        pose = linear_pose_interpolation(g.start_pose, g.end_pose, a)
        ant = [s + (e - s) * a for s, e in zip(g.start_ant, g.end_ant)]
        return pose, ant, g.start_yaw + (g.end_yaw - g.start_yaw) * a


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
    publish_tf, base_frame
        Broadcast on /tf the transforms robot_state_publisher cannot compute from
        joint_states: base_frame -> head (forward kinematics) and base_frame -> rod_1..6
        (default true, 'base_link'). See the reachy_mini_description package.

    Topics
    ------
    joint_states (sensor_msgs/JointState), ~/head_pose (geometry_msgs/PoseStamped), /tf: published.
    ~/target_head_pose (PoseStamped), ~/target_antennas (reachy_mini_msgs/AntennasTarget),
    ~/target_body_yaw (std_msgs/Float64): subscribed.

    Skills (ROS4HRI, under the `skills_namespace` parameter, default /skill)
    -----------------------------------------------------------------------
    replay_motion (motions_skills/action/ReplayMotion): play a recorded move by name.
    list_motions (motions_skills/srv/ListMotions): the names that replay_motion accepts.
    wake_up (motions_skills/action/ReplayMotion): torque on, the move `name` if not
    empty, then the rest pose.
    go_to_sleep (std_skills/action/Empty): the sleep pose (motors stay on).

    The C++ driver implements the full skill set (trajectories, look_at); this
    Python driver is a reference implementation limited to the motion skills above.

    """

    def __init__(self, bus: Optional[MotorBus] = None, **node_kwargs) -> None:
        """Open the bus, start the loop (`node_kwargs` go to rclpy.Node)."""
        super().__init__('reachy_mini_driver', **node_kwargs)
        self.declare_parameter('serial_port', '/dev/ttyAMA3')
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('dataset', DEFAULT_DATASET)
        self.declare_parameter('wake_up_on_start', True)
        self.declare_parameter('sleep_on_shutdown', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('skills_namespace', '/skill')
        self.declare_parameter('extra_datasets', [''])
        self.declare_parameter('motion_initial_goto', 0.5)

        port = self.get_parameter('serial_port').value
        if bus is not None:
            self.bus = bus
        elif port == 'fake':
            self.bus = FakeBus()
        else:
            try:
                self.bus = RealBus(port)
            except Exception as e:
                raise RuntimeError(
                    f'cannot open the servo bus on {port}: {e}\n'
                    'The servo bus is only there on the robot. To run without hardware, use the '
                    'simulated bus:\n'
                    '  ros2 launch reachy_mini driver.launch.py serial_port:=fake\n'
                    '  ros2 run reachy_mini_driver driver_node --ros-args -p serial_port:=fake'
                ) from e
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
        self.base_frame = self.get_parameter('base_frame').value
        self.geometry = PlatformGeometry()
        self.tf_broadcaster = (TransformBroadcaster(self)
                               if self.get_parameter('publish_tf').value else None)
        self.create_subscription(PoseStamped, '~/target_head_pose', self._on_target_pose, 10)
        self.create_subscription(AntennasTarget, '~/target_antennas', self._on_target_antennas, 10)
        self.create_subscription(Float64, '~/target_body_yaw', self._on_target_body_yaw, 10)

        self.extra_datasets = [d for d in self.get_parameter('extra_datasets').value if d]
        ns = self.get_parameter('skills_namespace').value
        self._cb_group = ReentrantCallbackGroup()
        self._move_priority = 0
        self._action = ActionServer(
            self, ReplayMotion, ns + '/replay_motion',
            execute_callback=self._execute_replay_motion,
            goal_callback=self._accept_motion_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._cb_group)
        self._wake_up_action = ActionServer(
            self, ReplayMotion, ns + '/wake_up',
            execute_callback=self._execute_wake_up,
            goal_callback=self._accept_motion_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._cb_group)
        self._sleep_action = ActionServer(
            self, EmptySkill, ns + '/go_to_sleep',
            execute_callback=self._execute_go_to_sleep,
            goal_callback=self._accept_motion_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._cb_group)
        self._list_service = self.create_service(
            ListMotions, ns + '/list_motions', self._list_motions, callback_group=self._cb_group)

        self.bus.set_torque(True)
        if self.get_parameter('wake_up_on_start').value:
            self.goto(INIT_HEAD_POSE, INIT_ANTENNAS, duration=2.0, body_yaw=0.0)

        period = 1.0 / self.get_parameter('rate').value
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(f'driver ready on {port} at {1 / period:.0f} Hz')

    # ------------------------------------------------------------------ targets
    def goto(self, head_pose, antennas, duration: float,
             body_yaw: Optional[float] = None) -> None:
        """Smoothly move to a head pose, antenna positions and (if given) body yaw."""
        with self._lock:
            end_yaw = self._target_body_yaw if body_yaw is None else body_yaw
            self._goto = Goto(self._target_pose, np.array(head_pose),
                              self._target_antennas, antennas,
                              self._target_body_yaw, end_yaw, duration)

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
                pose, ant, yaw, done = self._goto.evaluate()
                self._target_pose, self._target_antennas, self._target_body_yaw = pose, ant, yaw
                if done:
                    self._goto = None

            goals = self._compute_goals()

        js = JointState()
        js.header.stamp = now
        js.name = MOTOR_NAMES
        js.position = positions
        self.joint_pub.publish(js)
        self.pose_pub.publish(pose_to_msg(self._head_pose, now, self.base_frame))
        if self.tf_broadcaster is not None:
            transforms = [pose_to_transform(self.geometry.head_in_base(self._head_pose), now,
                                            self.base_frame, 'head')]
            rods = self.geometry.rods_in_base(positions[1:7], self._head_pose)
            transforms += [pose_to_transform(rod, now, self.base_frame, f'rod_{i + 1}')
                           for i, rod in enumerate(rods)]
            self.tf_broadcaster.sendTransform(transforms)

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

    # --------------------------------------------------------------- skills
    def _all_motion_names(self):
        names = set(self.library.names())
        for dataset in self.extra_datasets:
            try:
                names.update(self.library.names(dataset))
            except Exception as e:  # noqa: BLE001 - a bad extra library must not break listing
                self.get_logger().warning(f"cannot list dataset '{dataset}': {e}")
        return sorted(names)

    def _list_motions(self, request, response):
        """ROS4HRI list_motions: the names replay_motion accepts."""
        try:
            response.motions = self._all_motion_names()
            response.result.error_code = Result.ROS_ENOERR
        except Exception as e:  # noqa: BLE001
            response.result.error_code = Result.ROS_EOTHER
            response.result.error_msg = str(e)
        return response

    def _accept_motion_goal(self, goal):
        """Reject a goal whose priority is lower than the running motion's (std_skills/Meta)."""
        with self._lock:
            running = self._move is not None
        if running and goal.meta.priority < self._move_priority:
            self.get_logger().warning(
                'replay_motion goal rejected: a higher-priority motion is running')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _find_move(self, name: str):
        """Return the move `name`, from the default library or the extra datasets."""
        if not name:
            raise KeyError('empty motion name')
        try:
            return self.library.get(name)
        except KeyError:
            for dataset in self.extra_datasets:
                try:
                    return self.library.get(name, dataset)
                except KeyError:
                    pass
        raise KeyError(f"unknown motion '{name}'")

    def _goto_move(self, head_pose, antennas, body_yaw: float, duration: float) -> GotoMove:
        """Return a goto from the current targets, as a move."""
        with self._lock:
            return GotoMove(self._target_pose, np.array(head_pose), self._target_antennas,
                            antennas, self._target_body_yaw, body_yaw, duration)

    def _play(self, goal_handle, move, name: str, priority: int, feedback,
              initial_goto: float) -> int:
        """Play `move` in the motion slot until it ends; return the std_skills error code."""
        # preempt whatever is playing (the goal callback already filtered lower priorities)
        with self._lock:
            preempted = self._move is not None
            self._move_cancelled = True
            self._move_priority = priority
        if preempted:
            self._move_done.wait(timeout=1.0)
        # 1. reach the first pose of the move (so the move does not start with a jump)
        if initial_goto > 0:
            head0, ant0, yaw0 = move.evaluate(0.0)
            self.goto(head0, list(ant0), initial_goto, yaw0)
            time.sleep(initial_goto)
        # 2. play, ticking feedback until the loop says it is over
        with self._lock:
            self._goto = None
            self._move_cancelled = False
            self._move_done.clear()
            self._move = (move, time.monotonic())
            this_move = self._move
        while not self._move_done.wait(timeout=0.1):
            if goal_handle.is_cancel_requested:
                with self._lock:
                    self._move_cancelled = True
                self._move_done.wait(timeout=1.0)
                return Result.ROS_ECANCELED
            with self._lock:
                if self._move is not this_move:      # replaced by a newer goal
                    break
                elapsed = time.monotonic() - self._move[1] if self._move else move.duration
            feedback.feedback.data_float = float(min(1.0, elapsed / move.duration))
            feedback.feedback.data_str = name
            goal_handle.publish_feedback(feedback)
        with self._lock:
            if self._move is not None and self._move is not this_move:
                return Result.ROS_EINTR
        return Result.ROS_ENOERR

    @staticmethod
    def _finish(goal_handle, result, code: int):
        result.result.error_code = code
        if code == Result.ROS_ENOERR:
            goal_handle.succeed()
        elif code == Result.ROS_ECANCELED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _execute_replay_motion(self, goal_handle):
        """ROS4HRI replay_motion: play a recorded move by name."""
        goal = goal_handle.request
        result = ReplayMotion.Result()
        try:
            move = self._find_move(goal.name)
        except KeyError as e:   # the spec wants the list of motions back
            result.result.error_code = Result.ROS_EBADMSG
            result.result.error_msg = f'{e}; available: {", ".join(self._all_motion_names())}'
            goal_handle.abort()
            return result

        self.get_logger().info(
            f"playing '{goal.name}' ({move.duration:.1f} s) for {goal.meta.caller or 'unknown'}")
        code = self._play(goal_handle, move, goal.name, goal.meta.priority,
                          ReplayMotion.Feedback(),
                          self.get_parameter('motion_initial_goto').value)
        return self._finish(goal_handle, result, code)

    def _execute_wake_up(self, goal_handle):
        """wake_up: torque on, the animation `name` if any, then the rest pose."""
        goal = goal_handle.request
        result = ReplayMotion.Result()
        animation = None
        if goal.name:
            try:
                animation = self._find_move(goal.name)
            except KeyError as e:
                result.result.error_code = Result.ROS_EBADMSG
                result.result.error_msg = f'{e}; available: {", ".join(self._all_motion_names())}'
                goal_handle.abort()
                return result
        self.get_logger().info(
            'waking up' + (f" with '{goal.name}'" if goal.name else '') +
            f" for {goal.meta.caller or 'unknown'}")
        self.bus.set_torque(True)
        feedback = ReplayMotion.Feedback()
        code = Result.ROS_ENOERR
        if animation is not None:
            code = self._play(goal_handle, animation, goal.name, goal.meta.priority, feedback,
                              self.get_parameter('motion_initial_goto').value)
        if code == Result.ROS_ENOERR:
            rest = self._goto_move(INIT_HEAD_POSE, INIT_ANTENNAS, 0.0, 2.0)
            code = self._play(goal_handle, rest, 'wake_up', goal.meta.priority, feedback, 0.0)
        return self._finish(goal_handle, result, code)

    def _execute_go_to_sleep(self, goal_handle):
        """go_to_sleep: the sleep pose, body centred (the motors stay on)."""
        goal = goal_handle.request
        self.get_logger().info(f"going to sleep for {goal.meta.caller or 'unknown'}")
        sleep = self._goto_move(SLEEP_HEAD_POSE, SLEEP_ANTENNAS, 0.0, 2.0)
        code = self._play(goal_handle, sleep, 'go_to_sleep', goal.meta.priority,
                          EmptySkill.Feedback(), 0.0)
        return self._finish(goal_handle, EmptySkill.Result(), code)

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
            self.goto(SLEEP_HEAD_POSE, SLEEP_ANTENNAS, duration=2.0, body_yaw=0.0)
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
    try:
        node = DriverNode()
    except RuntimeError as e:
        get_logger('reachy_mini_driver').fatal(str(e))
        rclpy.shutdown()
        sys.exit(1)
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
