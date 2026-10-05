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

TEST(Moves, ResolveDatasetFromPackage)
{
  // Fake install prefix: the ament index marker and the package's share directory
  namespace fs = std::filesystem;
  const fs::path prefix = fs::temp_directory_path() / "rmd_ament_prefix_test";
  fs::create_directories(prefix / "share" / "ament_index" / "resource_index" / "packages");
  std::ofstream(prefix / "share" / "ament_index" / "resource_index" / "packages" / "fake_moves");
  fs::create_directories(prefix / "share" / "fake_moves" / "emotions");
  fs::copy_file(
    fs::path(TEST_MOVES_DIR) / "nod.json", prefix / "share" / "fake_moves" / "emotions" / "nod.json",
    fs::copy_options::overwrite_existing);
  const char * previous = std::getenv("AMENT_PREFIX_PATH");
  const std::string saved = previous ? previous : "";
  setenv("AMENT_PREFIX_PATH", (prefix.string() + (previous ? ":" + saved : "")).c_str(), 1);

  EXPECT_EQ(
    MoveLibrary::resolveDataset("package://fake_moves/emotions"),
    (prefix / "share" / "fake_moves" / "emotions").string());
  MoveLibrary lib("package://fake_moves/emotions");
  EXPECT_NO_THROW(lib.get("nod"));
  EXPECT_THROW(MoveLibrary::resolveDataset("package://fake_moves/nothing"), std::runtime_error);
  EXPECT_THROW(MoveLibrary::resolveDataset("package://no_such_package/x"), std::runtime_error);

  if (previous) {
    setenv("AMENT_PREFIX_PATH", saved.c_str(), 1);
  } else {
    unsetenv("AMENT_PREFIX_PATH");
  }
  fs::remove_all(prefix);
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

TEST(Kinematics, RodsHaveTheRodLength)
{
  Kinematics kin;
  Pose target = Pose::Identity();
  target.translation() = Eigen::Vector3d(0.01, 0.0, 0.015);
  target.linear() = (Eigen::AngleAxisd(0.17, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(-0.14, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.26, Eigen::Vector3d::UnitZ())).toRotationMatrix();
  const auto joints = kin.ik(target, 0.0);
  ASSERT_TRUE(joints.has_value());
  for (double l : kin.rodLengths(*joints, target)) {
    EXPECT_NEAR(l, kin.rodLength(), 1e-4);
  }
  const auto rods = kin.rodsInBase(*joints, target);
  ASSERT_EQ(rods.size(), 6u);
  EXPECT_NEAR(rods[0].rotation().determinant(), 1.0, 1e-9);
  EXPECT_NEAR(kin.headInBase(Pose::Identity()).translation().z(), kin.headZOffset(), 1e-12);
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
