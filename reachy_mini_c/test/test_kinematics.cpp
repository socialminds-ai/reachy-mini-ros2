// Copyright 2026 SocialMinds Lab, IIIA-CSIC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Unit tests for the kinematics part of reachy_mini_c (no hardware needed).
// Reference values come from the Python SDK (AnalyticalKinematics), see
// ../tools/check_reachy_mini_c_against_sdk.py.
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "reachy_mini_c/reachy_mini_c.h"

static const double kIdentity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

TEST(Kinematics, VersionAndCreate) {
  ASSERT_STRNE(rmc_version(), "");
  RmcKinematics *k = rmc_kinematics_new();
  ASSERT_NE(k, nullptr) << rmc_last_error();
  rmc_kinematics_free(k);
}

TEST(Kinematics, IdentityPoseMatchesSdk) {
  RmcKinematics *k = rmc_kinematics_new();
  ASSERT_NE(k, nullptr);
  double joints[RMC_NUM_JOINTS];
  ASSERT_EQ(rmc_kinematics_ik(k, kIdentity, 0.0, joints), 0) << rmc_last_error();
  // SDK: ik(eye(4), body_yaw=0) -> [0, 0.6265, -0.6265, 0.6265, -0.6265, 0.6265, -0.6265]
  EXPECT_NEAR(joints[0], 0.0, 1e-9);
  for (int i = 1; i < 7; ++i) {
    EXPECT_NEAR(std::fabs(joints[i]), 0.6265, 1e-3) << "joint " << i;
    EXPECT_EQ(joints[i] > 0, i % 2 == 1) << "joint " << i;
  }
  rmc_kinematics_free(k);
}

TEST(Kinematics, BodyYawIsPassedThroughWithinLimits) {
  RmcKinematics *k = rmc_kinematics_new();
  double joints[RMC_NUM_JOINTS];
  ASSERT_EQ(rmc_kinematics_ik(k, kIdentity, 0.3, joints), 0) << rmc_last_error();
  EXPECT_NEAR(joints[0], 0.3, 1e-9);
  ASSERT_EQ(rmc_kinematics_ik_fixed_yaw(k, kIdentity, 0.3, joints), 0) << rmc_last_error();
  EXPECT_NEAR(joints[0], 0.3, 1e-9);
  rmc_kinematics_free(k);
}

TEST(Kinematics, ForwardOfInverseIsIdentityForSmallMotion) {
  RmcKinematics *k = rmc_kinematics_new();
  // 1 cm up, no rotation
  double pose[16];
  std::memcpy(pose, kIdentity, sizeof(pose));
  pose[11] = 0.01;  // z translation (row 2, col 3)
  double joints[RMC_NUM_JOINTS], back[16];
  ASSERT_EQ(rmc_kinematics_ik(k, pose, 0.0, joints), 0) << rmc_last_error();
  ASSERT_EQ(rmc_kinematics_reset_fk(k, kIdentity), 0);
  ASSERT_EQ(rmc_kinematics_fk(k, joints, 20, back), 0) << rmc_last_error();
  for (int i = 0; i < 16; ++i) {
    EXPECT_NEAR(back[i], pose[i], 1e-4) << "element " << i;
}
  rmc_kinematics_free(k);
}

TEST(Kinematics, UnreachablePoseIsAnError) {
  RmcKinematics *k = rmc_kinematics_new();
  double pose[16];
  std::memcpy(pose, kIdentity, sizeof(pose));
  pose[11] = 0.5;  // 50 cm up: impossible
  double joints[RMC_NUM_JOINTS];
  EXPECT_NE(rmc_kinematics_ik(k, pose, 0.0, joints), 0);
  EXPECT_STRNE(rmc_last_error(), "");
  rmc_kinematics_free(k);
}

TEST(Kinematics, NullHandlesAreErrors) {
  double joints[RMC_NUM_JOINTS];
  EXPECT_NE(rmc_kinematics_ik(nullptr, kIdentity, 0.0, joints), 0);
  EXPECT_EQ(rmc_motors_open(nullptr), nullptr);
}
