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

#include "reachy_mini_driver_cpp/moves.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace reachy_mini_driver
{

RecordedMove RecordedMove::fromFile(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("cannot open move file " + path);
  }
  const auto json = nlohmann::json::parse(in);
  const auto & times = json.at("time");
  const auto & data = json.at("set_target_data");
  if (times.size() != data.size() || times.empty()) {
    throw std::runtime_error("malformed move file " + path);
  }
  std::vector<MoveSample> samples;
  samples.reserve(times.size());
  for (std::size_t i = 0; i < times.size(); ++i) {
    MoveSample s;
    s.time = times[i].get<double>();
    const auto & d = data[i];
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    if (d.contains("head") && !d["head"].is_null()) {
      for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
          m(r, c) = d["head"][r][c].get<double>();
        }
      }
    }
    s.head.matrix() = m;
    s.antennas = {0.0, 0.0};
    if (d.contains("antennas") && !d["antennas"].is_null()) {
      s.antennas = {d["antennas"][0].get<double>(), d["antennas"][1].get<double>()};
    }
    s.body_yaw = (d.contains("body_yaw") &&
      !d["body_yaw"].is_null()) ? d["body_yaw"].get<double>() : 0.0;
    samples.push_back(s);
  }
  return RecordedMove(std::move(samples));
}

RecordedMove::RecordedMove(std::vector<MoveSample> samples)
: samples_(std::move(samples))
{
  if (samples_.empty()) {
    throw std::invalid_argument("a move needs at least one sample");
  }
}

MoveSample RecordedMove::evaluate(double t) const
{
  t = std::clamp(t, samples_.front().time, samples_.back().time);
  // first sample with time > t
  const auto next = std::upper_bound(
    samples_.begin(), samples_.end(), t,
    [](double value, const MoveSample & s) {return value < s.time;});
  if (next == samples_.begin()) {
    return samples_.front();
  }
  if (next == samples_.end()) {
    return samples_.back();
  }
  const auto & a = *(next - 1);
  const auto & b = *next;
  const double alpha = (b.time - a.time) > 0 ? (t - a.time) / (b.time - a.time) : 0.0;
  MoveSample s;
  s.time = t;
  s.head = interpolate(a.head, b.head, alpha);
  s.antennas = {a.antennas[0] + (b.antennas[0] - a.antennas[0]) * alpha,
    a.antennas[1] + (b.antennas[1] - a.antennas[1]) * alpha};
  s.body_yaw = a.body_yaw + (b.body_yaw - a.body_yaw) * alpha;
  return s;
}

MoveLibrary::MoveLibrary(std::string default_dataset)
: default_dataset_(std::move(default_dataset)) {}

std::string MoveLibrary::resolveDataset(const std::string & dataset)
{
  namespace fs = std::filesystem;
  if (fs::is_directory(dataset)) {
    return dataset;
  }
  // HuggingFace cache layout: <cache>/hub/datasets--<org>--<name>/snapshots/<sha>/
  const char * home = std::getenv("HOME");
  const char * hf_home = std::getenv("HF_HOME");
  fs::path cache = hf_home ? fs::path(hf_home) : fs::path(home ? home : "/root") / ".cache" /
    "huggingface";
  // org/name -> datasets--org--name (dashes inside org and name are kept as is)
  std::string folder = "datasets--" + dataset;
  for (std::size_t pos = folder.find('/'); pos != std::string::npos;
    pos = folder.find('/', pos + 2))
  {
    folder.replace(pos, 1, "--");
  }
  const fs::path repo = cache / "hub" / folder;
  const fs::path ref = repo / "refs" / "main";
  std::ifstream in(ref);
  std::string sha;
  if (!(in >> sha)) {
    throw std::runtime_error(
      "dataset '" + dataset + "' is not in the HuggingFace cache (" + repo.string() +
      "); download it first, e.g. with the Python SDK");
  }
  return (repo / "snapshots" / sha).string();
}

std::shared_ptr<const RecordedMove> MoveLibrary::get(
  const std::string & move,
  const std::string & dataset)
{
  const std::string ds = dataset.empty() ? default_dataset_ : dataset;
  const std::string key = ds + "/" + move;
  const auto it = cache_.find(key);
  if (it != cache_.end()) {
    return it->second;
  }
  namespace fs = std::filesystem;
  const fs::path dir = resolveDataset(ds);
  fs::path file = dir / (move + ".json");
  if (!fs::exists(file)) {
    file = dir / "data" / (move + ".json");   // newer datasets keep moves in data/
  }
  if (!fs::exists(file)) {
    throw std::runtime_error("move '" + move + "' not found in " + ds);
  }
  auto loaded = std::make_shared<const RecordedMove>(RecordedMove::fromFile(file.string()));
  cache_[key] = loaded;
  return loaded;
}

}  // namespace reachy_mini_driver
