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
The servo bus, behind a small interface.

`RealBus` talks to the 9 Dynamixel servos through Pollen's
`reachy_mini_motor_controller` (Rust). `FakeBus` is a perfect servo: the
position you write is the position you read. Tests use it; so can you, on a
laptop without the robot.
"""

import abc
from typing import List

MOTOR_NAMES = [
    'body_yaw',
    'stewart_1', 'stewart_2', 'stewart_3', 'stewart_4', 'stewart_5', 'stewart_6',
    'right_antenna', 'left_antenna',
]


class MotorBus(abc.ABC):
    """9 motors: [body_yaw, stewart_1..6, right_antenna, left_antenna], radians."""

    @abc.abstractmethod
    def read_positions(self) -> List[float]:
        """Read the present position of the 9 motors."""

    @abc.abstractmethod
    def write_goal_positions(self, goals: List[float]) -> None:
        """Write the goal position of the 9 motors."""

    @abc.abstractmethod
    def set_torque(self, enabled: bool) -> None:
        """Enable or disable torque on all motors."""

    def close(self) -> None:
        """Release the bus (optional)."""


class FakeBus(MotorBus):
    """An ideal bus: positions follow goals instantly. Records everything."""

    def __init__(self, initial: List[float] | None = None) -> None:
        """Start at `initial` (default: all zeros)."""
        self.positions = list(initial) if initial else [0.0] * 9
        self.torque = False
        self.writes = 0

    def read_positions(self) -> List[float]:
        """Return the last goals written."""
        return list(self.positions)

    def write_goal_positions(self, goals: List[float]) -> None:
        """Move instantly to the goals (only if torque is on, like real servos)."""
        if len(goals) != 9:
            raise ValueError('expected 9 goal positions')
        self.writes += 1
        if self.torque:
            self.positions = [float(g) for g in goals]

    def set_torque(self, enabled: bool) -> None:
        """Remember the torque state."""
        self.torque = enabled


class RealBus(MotorBus):
    """The real thing, through Pollen's Rust motor controller."""

    def __init__(self, serial_port: str) -> None:
        """Open the bus on `serial_port` (e.g. /dev/ttyAMA3)."""
        from reachy_mini_motor_controller import ReachyMiniMotorController
        self.controller = ReachyMiniMotorController(serial_port)

    def read_positions(self) -> List[float]:
        """Sync-read the 9 present positions."""
        return list(self.controller.read_all_positions())

    def write_goal_positions(self, goals: List[float]) -> None:
        """Sync-write the 9 goal positions."""
        self.controller.set_all_goal_positions(list(goals))

    def set_torque(self, enabled: bool) -> None:
        """Torque on/off on all motors."""
        if enabled:
            self.controller.enable_torque()
        else:
            self.controller.disable_torque()
