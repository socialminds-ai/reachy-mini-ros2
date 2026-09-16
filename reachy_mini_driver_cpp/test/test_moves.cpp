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

// Tests of the move loader/evaluator and of the kinematics wrapper (no ROS).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#include "reachy_mini_driver_cpp/kinematics.hpp"
#include "reachy_mini_driver_cpp/moves.hpp"

using reachy_mini_driver::Kinematics;
using reachy_mini_driver::MoveLibrary;
using reachy_mini_driver::Pose;
using reachy_mini_driver::RecordedMove;

TEST(Moves, LoadAndEvaluate)
{
  const auto move = RecordedMove::fromFile(std::string(TEST_MOVES_DIR) + "/nod.json");
  EXPECT_NEAR(move.duration(), 1.0, 1e-9);
  const auto start = move.evaluate(0.0);
  EXPECT_NEAR(start.antennas[0], 0.0, 1e-9);
  const auto mid = move.evaluate(0.5);          // peak of the nod
  EXPECT_NEAR(mid.antennas[0], -0.3, 1e-6);
  EXPECT_NEAR(mid.antennas[1], 0.3, 1e-6);
  const double pitch = std::asin(-mid.head.rotation()(2, 0));
  EXPECT_NEAR(pitch, 15.0 * M_PI / 180.0, 1e-3);
  const auto between = move.evaluate(0.51);     // interpolated between two samples
  EXPECT_LT(between.antennas[0], -0.29);
  EXPECT_EQ(move.evaluate(5.0).time, 1.0);      // clamped
}

TEST(Moves, ResolveDatasetFromHuggingFaceCache)
{
  // Fake cache: <HF_HOME>/hub/datasets--org--name/{refs/main, snapshots/<sha>/}
  namespace fs = std::filesystem;
  const fs::path home = fs::temp_directory_path() / "rmd_hf_cache_test";
  const fs::path repo = home / "hub" / "datasets--pollen-robotics--reachy-mini-emotions-library";
  fs::create_directories(repo / "refs");
  fs::create_directories(repo / "snapshots" / "abc123");
  std::ofstream(repo / "refs" / "main") << "abc123";
  setenv("HF_HOME", home.c_str(), 1);
  EXPECT_EQ(
    MoveLibrary::resolveDataset("pollen-robotics/reachy-mini-emotions-library"),
    (repo / "snapshots" / "abc123").string());
  EXPECT_THROW(MoveLibrary::resolveDataset("nobody/nothing"), std::runtime_error);
  unsetenv("HF_HOME");
  fs::remove_all(home);
}

TEST(Moves, LibraryFromDirectory)
{
  MoveLibrary lib(TEST_MOVES_DIR);
  EXPECT_NO_THROW(lib.get("nod"));
  EXPECT_THROW(lib.get("does_not_exist"), std::runtime_error);
  EXPECT_THROW(lib.get("nod", "/nonexistent/dataset"), std::runtime_error);
}

TEST(Kinematics, IkFkRoundTrip)
{
  Kinematics kin;
  Pose target = Pose::Identity();
  target.translation().z() = 0.01;
  const auto joints = kin.ik(target, 0.0);
  ASSERT_TRUE(joints.has_value());
  kin.resetFk(Pose::Identity());
  const auto back = kin.fk(*joints, 20);
  EXPECT_TRUE(back.isApprox(target, 1e-4));
  Pose impossible = Pose::Identity();
  impossible.translation().z() = 0.5;
  EXPECT_FALSE(kin.ik(impossible, 0.0).has_value());
}

TEST(Kinematics, InterpolateHalfway)
{
  Pose a = Pose::Identity();
  Pose b = Pose::Identity();
  b.translation().x() = 0.02;
  b.linear() = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto h = reachy_mini_driver::interpolate(a, b, 0.5);
  EXPECT_NEAR(h.translation().x(), 0.01, 1e-12);
  EXPECT_NEAR(Eigen::AngleAxisd(h.rotation()).angle(), 0.2, 1e-9);
}
