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
"""The rod geometry must be consistent with the analytical kinematics."""

import numpy as np
import pytest
from reachy_mini.kinematics.analytical_kinematics import AnalyticalKinematics
from reachy_mini.utils import create_head_pose
from reachy_mini_driver.geometry import PlatformGeometry


@pytest.fixture
def geometry():
    return PlatformGeometry()


def test_rest_pose_rods_have_the_rod_length(geometry):
    kin = AnalyticalKinematics(automatic_body_yaw=True)
    pose = np.eye(4)
    joints = kin.ik(pose, body_yaw=0.0)
    lengths = geometry.rod_lengths(joints[1:], pose)
    assert np.allclose(lengths, geometry.rod_length, atol=1e-4), lengths


def test_moved_pose_rods_have_the_rod_length(geometry):
    kin = AnalyticalKinematics(automatic_body_yaw=False)
    pose = create_head_pose(x=0.01, z=0.015, roll=10, pitch=-8, yaw=15, degrees=True)
    joints = kin.ik(pose, body_yaw=0.0)
    assert not np.any(np.isnan(joints))
    lengths = geometry.rod_lengths(joints[1:], pose)
    assert np.allclose(lengths, geometry.rod_length, atol=1e-4), lengths
    for rod in geometry.rods_in_base(joints[1:], pose):
        # proper rotation, origin at the horn tip height range of the platform
        assert np.isclose(np.linalg.det(rod[:3, :3]), 1.0)


def test_head_in_base_adds_rest_height(geometry):
    t = geometry.head_in_base(np.eye(4))
    assert np.isclose(t[2, 3], geometry.head_z_offset)
