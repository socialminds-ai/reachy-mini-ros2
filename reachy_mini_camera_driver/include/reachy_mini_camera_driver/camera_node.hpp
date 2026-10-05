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

#ifndef REACHY_MINI_CAMERA_DRIVER__CAMERA_NODE_HPP_
#define REACHY_MINI_CAMERA_DRIVER__CAMERA_NODE_HPP_

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "camera_info_manager/camera_info_manager.hpp"
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include "foxglove_msgs/msg/compressed_video.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace reachy_mini_camera_driver
{

// What the GStreamer pipeline is built from. Everything in here needs a
// pipeline restart to change; the libcamera controls (focus, exposure value)
// are applied live and are not part of it.
struct StreamSettings
{
  std::string source;     // "libcamera", "v4l2:<device>" or "test"
  std::string camera_name;
  std::string encoder;    // "v4l2h264enc" (Raspberry Pi hardware), "x264enc", "openh264enc"
  int width = 1280;
  int height = 720;
  int framerate = 30;
  int bitrate = 2000000;  // bit/s
  int keyframe_interval = 60;  // frames
  std::string h264_profile = "high";
  std::string field_of_view = "wide";  // libcamera: "wide" or "narrow" sensor mode
};

// The gst-launch description of the capture + encode pipeline. Exposed for tests.
std::string build_pipeline_description(const StreamSettings & s);

// Camera node: libcamera (or a V4L2 / test source) -> tee -> hardware H.264 ->
// ffmpeg_image_transport packets and foxglove CompressedVideo, and -> raw YUY2
// sensor_msgs/Image. Each output only costs something while it has subscribers;
// with `lazy` the camera itself is stopped when nobody listens.
class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraNode() override;

private:
  bool start_pipeline();
  void stop_pipeline();
  void on_timer();
  bool poll_bus();  // logs the pipeline messages; true on error or end of stream
  void update_valves();
  void apply_camera_controls();
  void request_keyframe();

  static GstFlowReturn on_encoded_sample(GstAppSink * sink, gpointer self);
  static GstFlowReturn on_raw_sample(GstAppSink * sink, gpointer self);
  void handle_encoded(GstSample * sample);
  void handle_raw(GstSample * sample);
  builtin_interfaces::msg::Time stamp_of(GstSample * sample);
  void publish_camera_info(const std_msgs::msg::Header & header, int width, int height);
  void load_calibration();

  rcl_interfaces::msg::SetParametersResult validate_parameters(
    const std::vector<rclcpp::Parameter> & parameters);
  void on_parameters_set(const std::vector<rclcpp::Parameter> & parameters);
  StreamSettings read_settings();

  // pipeline, only touched from the executor thread
  GstElement * pipeline_ = nullptr;
  GstElement * source_ = nullptr;
  GstElement * encoded_valve_ = nullptr;
  GstElement * raw_valve_ = nullptr;
  GstElement * encoded_sink_ = nullptr;
  StreamSettings settings_;
  bool restart_requested_ = false;
  bool controls_changed_ = false;
  std::chrono::steady_clock::time_point last_wanted_;
  std::chrono::steady_clock::time_point last_start_attempt_;
  std::chrono::steady_clock::time_point last_stream_change_;
  size_t encoded_subscribers_seen_ = 0;
  std::chrono::steady_clock::time_point first_keyframe_request_;
  bool second_keyframe_pending_ = false;

  // shared with the GStreamer streaming threads; the subscriber counts are
  // refreshed by the timer so the frame callbacks don't query the graph
  std::atomic<size_t> raw_subscribers_{0};
  std::atomic<size_t> ffmpeg_subscribers_{0};
  std::atomic<size_t> foxglove_subscribers_{0};
  std::atomic<size_t> camera_info_subscribers_{0};
  std::atomic<bool> wait_for_keyframe_{true};
  std::atomic<bool> raw_active_{false};
  std::atomic<uint64_t> packet_count_{0};
  std::string frame_id_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  std::string calibration_url_;  // the one camera_info_manager_ holds

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;
  rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr ffmpeg_pub_;
  rclcpp::Publisher<foxglove_msgs::msg::CompressedVideo>::SharedPtr foxglove_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr validate_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_handle_;
};

}  // namespace reachy_mini_camera_driver

#endif  // REACHY_MINI_CAMERA_DRIVER__CAMERA_NODE_HPP_
