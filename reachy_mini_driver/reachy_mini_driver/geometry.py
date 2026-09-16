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
Transforms of the moving parts that robot_state_publisher cannot compute.

The Stewart platform is a closed chain: the head pose comes from the driver's
forward kinematics, and each of the six rods hangs between a motor horn tip
(known from the motor angle) and an anchor on the head platform (known from the
head pose). This module turns those into the `base_link -> head` and
`base_link -> rod_i` transforms; the driver broadcasts them on /tf.

Geometry comes from the SDK's kinematics_data.json (the same file the
analytical kinematics use).
"""

from importlib.resources import files
import json
from typing import List, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R


class PlatformGeometry:
    """Motor frames, rod anchors and lengths of the Stewart platform."""

    def __init__(self) -> None:
        """Load the geometry from the reachy_mini SDK package data."""
        import reachy_mini
        data = json.loads(files(reachy_mini).joinpath('assets/kinematics_data.json').read_text())
        self.arm_length = float(data['motor_arm_length'])
        self.rod_length = float(data['rod_length'])
        self.head_z_offset = float(data['head_z_offset'])
        # T_motor_world in the SDK is the base frame seen from the motor: invert it
        self.t_base_motor = [np.linalg.inv(np.array(m['T_motor_world'])) for m in data['motors']]
        self.anchor_head = [np.array(m['branch_position']) for m in data['motors']]

    def head_in_base(self, head_pose: np.ndarray) -> np.ndarray:
        """Return the `head` frame in `base_link`: the FK pose plus the rest height."""
        t = np.array(head_pose, dtype=float)
        t[2, 3] += self.head_z_offset
        return t

    def rods_in_base(self, stewart_angles, head_pose: np.ndarray) -> List[np.ndarray]:
        """
        Pose of each rod frame in `base_link`.

        Origin at the horn tip, x axis towards the anchor on the platform (the
        mesh is centred at x = rod_length / 2). Rods are cylinders with ball
        joints at both ends, so the roll about their axis is arbitrary and we
        take the minimal rotation from +x to the rod direction.
        """
        t_base_head = self.head_in_base(head_pose)
        poses = []
        for i, theta in enumerate(stewart_angles):
            tip = self.t_base_motor[i] @ np.array(
                [self.arm_length * np.cos(theta), self.arm_length * np.sin(theta), 0.0, 1.0])
            anchor = t_base_head @ np.append(self.anchor_head[i], 1.0)
            axis = anchor[:3] - tip[:3]
            length = np.linalg.norm(axis)
            axis = axis / length if length > 1e-9 else np.array([1.0, 0.0, 0.0])
            rot, _ = R.align_vectors([axis], [[1.0, 0.0, 0.0]])
            pose = np.eye(4)
            pose[:3, :3] = rot.as_matrix()
            pose[:3, 3] = tip[:3]
            poses.append(pose)
        return poses

    def rod_lengths(self, stewart_angles, head_pose: np.ndarray) -> Tuple[float, ...]:
        """Distance horn tip <-> anchor of each rod (should equal rod_length)."""
        t_base_head = self.head_in_base(head_pose)
        out = []
        for i, theta in enumerate(stewart_angles):
            tip = self.t_base_motor[i] @ np.array(
                [self.arm_length * np.cos(theta), self.arm_length * np.sin(theta), 0.0, 1.0])
            anchor = t_base_head @ np.append(self.anchor_head[i], 1.0)
            out.append(float(np.linalg.norm(anchor[:3] - tip[:3])))
        return tuple(out)
