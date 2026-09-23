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

#include "reachy_mini_camera_driver/camera_node.hpp"

#include <gst/video/video.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace reachy_mini_camera_driver
{

namespace
{

// lazy mode: how long the camera keeps running after the last subscriber left
constexpr auto kIdleTimeout = std::chrono::seconds(5);
// delay between two attempts to open the camera
constexpr auto kRetryDelay = std::chrono::seconds(2);
// stream parameters take effect once they stop changing for this long, so
// that setting width then height restarts the camera once
constexpr auto kSettleDelay = std::chrono::seconds(1);
// A new subscriber gets a keyframe at once, and another one this much later:
// the node sees the subscriber before the subscriber has finished discovering
// the node, and whatever is published in between is lost.
constexpr auto kSecondKeyframeDelay = std::chrono::milliseconds(500);
// the Raspberry Pi 4 / CM4 hardware encoder stops at 1080p
constexpr int kHardwareEncoderMaxWidth = 1920;
constexpr int kHardwareEncoderMaxHeight = 1080;

const std::set<std::string> kRestartParameters = {
  "width", "height", "framerate", "bitrate", "keyframe_interval", "h264_profile"};
const std::set<std::string> kLiveParameters = {
  "autofocus_mode", "lens_position", "exposure_value"};

std::string caps_profile(const std::string & profile)
{
  // x264enc and v4l2h264enc both call it constrained-baseline in caps
  return profile == "baseline" ? "constrained-baseline" : profile;
}

rcl_interfaces::msg::ParameterDescriptor describe(
  const std::string & description, bool read_only = false)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = description;
  d.read_only = read_only;
  return d;
}

rcl_interfaces::msg::ParameterDescriptor describe_int(
  const std::string & description, int64_t from, int64_t to, uint64_t step = 1)
{
  auto d = describe(description);
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = from;
  range.to_value = to;
  range.step = step;
  d.integer_range.push_back(range);
  return d;
}

rcl_interfaces::msg::ParameterDescriptor describe_float(
  const std::string & description, double from, double to)
{
  auto d = describe(description);
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = from;
  range.to_value = to;
  d.floating_point_range.push_back(range);
  return d;
}

}  // namespace

std::string build_pipeline_description(const StreamSettings & s)
{
  std::ostringstream raw_caps;
  raw_caps << "video/x-raw,format=YUY2,width=" << s.width << ",height=" << s.height
           << ",framerate=" << s.framerate << "/1";

  std::ostringstream d;
  if (s.source == "libcamera") {
    d << "libcamerasrc name=source";
    if (!s.camera_name.empty()) {
      d << " camera-name=\"" << s.camera_name << "\"";
    }
    // the ISP scales and converts; the colorimetry is what v4l2h264enc accepts
    d << " ! " << raw_caps.str() << ",colorimetry=bt709,interlace-mode=progressive";
  } else if (s.source.rfind("v4l2:", 0) == 0) {
    d << "v4l2src name=source device=" << s.source.substr(5)
      << " ! decodebin ! videoconvert ! videoscale ! videorate ! " << raw_caps.str();
  } else {
    d << "videotestsrc name=source is-live=true pattern=ball ! " << raw_caps.str();
  }
  d << " ! tee name=split";

  // Encoded branch. The valve is closed while nobody subscribes to the
  // packets, so the encoder is idle; a leaky queue drops frames rather than
  // stalling the camera if the encoder falls behind.
  d << " split. ! queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0"
    << " ! valve name=encoded_valve drop=true";
  if (s.encoder == "v4l2h264enc") {
    d << " ! v4l2h264enc";
    if (s.source == "libcamera") {
      // Encode libcamera's dmabufs in place. By default the encoder copies
      // every frame into its own buffers: 37 % of a CM4 core at 1080p30,
      // against 9 % for the whole camera pipeline without the copy.
      d << " output-io-mode=dmabuf-import";
    }
    d << " extra-controls=\"controls,repeat_sequence_header=1"
      << ",video_bitrate=" << s.bitrate
      << ",h264_i_frame_period=" << s.keyframe_interval << "\""
      << " ! video/x-h264,profile=" << caps_profile(s.h264_profile) << ",level=(string)4.1";
  } else if (s.encoder == "x264enc") {
    d << " ! videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast"
      << " bitrate=" << std::max(1, s.bitrate / 1000)
      << " key-int-max=" << s.keyframe_interval
      << " ! video/x-h264,profile=" << caps_profile(s.h264_profile);
  } else {
    d << " ! videoconvert ! openh264enc complexity=low"
      << " bitrate=" << s.bitrate << " gop-size=" << s.keyframe_interval;
  }
  // SPS/PPS before every keyframe, so a decoder can join mid-stream
  d << " ! h264parse config-interval=-1"
    << " ! video/x-h264,stream-format=byte-stream,alignment=au"
    << " ! appsink name=encoded_sink sync=false async=false max-buffers=4 drop=true";

  // Raw branch. Both appsinks are async=false: behind a closed valve a sink
  // never prerolls, which would hold the whole pipeline in PAUSED.
  d << " split. ! queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0"
    << " ! valve name=raw_valve drop=true"
    << " ! appsink name=raw_sink sync=false async=false max-buffers=2 drop=true";
  return d.str();
}

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("camera", options)
{
  gst_init(nullptr, nullptr);

  declare_parameter<std::string>(
    "source", "libcamera",
    describe("'libcamera' (the CSI camera), 'v4l2:<device>' or 'test' (videotestsrc)", true));
  declare_parameter<std::string>(
    "camera_name", "", describe("libcamera camera id; empty for the first camera", true));
  declare_parameter<std::string>(
    "encoder", "v4l2h264enc",
    describe(
      "H.264 encoder: 'v4l2h264enc' (Raspberry Pi hardware), 'x264enc' or 'openh264enc' "
      "(software, for development machines)", true));
  declare_parameter<std::string>("frame_id", "camera_optical", describe("frame of the images", true));
  declare_parameter<std::string>(
    "camera_info_url", "",
    describe("camera_info_manager URL of the calibration (e.g. file:///path/camera.yaml)", true));
  declare_parameter<bool>(
    "lazy", true, describe("stop the camera when no image topic has subscribers"));

  declare_parameter<int>("width", 1280, describe_int("image width (pixels, even)", 64, 4608, 2));
  declare_parameter<int>("height", 720, describe_int("image height (pixels, even)", 64, 2592, 2));
  declare_parameter<int>("framerate", 30, describe_int("frames per second", 1, 120));
  declare_parameter<int>(
    "bitrate", 2000000,
    describe_int("H.264 target bitrate (bit/s): the compression level", 100000, 25000000));
  declare_parameter<int>(
    "keyframe_interval", 60,
    describe_int(
      "frames between two H.264 keyframes (new subscribers also trigger one)", 1, 600));
  declare_parameter<std::string>(
    "h264_profile", "high", describe("H.264 profile: 'baseline', 'main' or 'high'"));

  declare_parameter<std::string>(
    "autofocus_mode", "continuous",
    describe("libcamera only: 'continuous', 'auto' (one scan per trigger) or 'manual'"));
  declare_parameter<double>(
    "lens_position", 1.0,
    describe_float(
      "libcamera only, autofocus_mode 'manual': focus distance in dioptres (1/m; 0 = infinity)",
      0.0, 32.0));
  declare_parameter<double>(
    "exposure_value", 0.0,
    describe_float("libcamera only: exposure compensation (EV, log2 steps)", -8.0, 8.0));

  // validate the values given at startup too
  for (const auto & p : get_parameters(
      {"encoder", "width", "height", "h264_profile", "autofocus_mode"}))
  {
    const auto result = validate_parameters({p});
    if (!result.successful) {
      throw std::invalid_argument(result.reason);
    }
  }
  settings_ = read_settings();
  frame_id_ = get_parameter("frame_id").as_string();

  camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, "reachy_mini_camera", get_parameter("camera_info_url").as_string());

  const auto qos = rclcpp::QoS(5);
  raw_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", qos);
  ffmpeg_pub_ = create_publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(
    "image_raw/ffmpeg", qos);
  foxglove_pub_ = create_publisher<foxglove_msgs::msg::CompressedVideo>("image_raw/h264", qos);
  camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos);

  validate_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & p) {return validate_parameters(p);});
  post_set_handle_ = add_post_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & p) {on_parameters_set(p);});

  timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {on_timer();});

  RCLCPP_INFO(
    get_logger(), "camera driver ready (source %s, encoder %s, %dx%d @ %d fps, %d bit/s)%s",
    settings_.source.c_str(), settings_.encoder.c_str(), settings_.width, settings_.height,
    settings_.framerate, settings_.bitrate,
    get_parameter("lazy").as_bool() ? "; the camera starts with the first subscriber" : "");
}

CameraNode::~CameraNode()
{
  stop_pipeline();
}

StreamSettings CameraNode::read_settings()
{
  StreamSettings s;
  s.source = get_parameter("source").as_string();
  s.camera_name = get_parameter("camera_name").as_string();
  s.encoder = get_parameter("encoder").as_string();
  s.width = get_parameter("width").as_int();
  s.height = get_parameter("height").as_int();
  s.framerate = get_parameter("framerate").as_int();
  s.bitrate = get_parameter("bitrate").as_int();
  s.keyframe_interval = get_parameter("keyframe_interval").as_int();
  s.h264_profile = get_parameter("h264_profile").as_string();
  return s;
}

rcl_interfaces::msg::SetParametersResult CameraNode::validate_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  const bool hardware =
    (has_parameter("encoder") ? get_parameter("encoder").as_string() : "") == "v4l2h264enc";
  for (const auto & p : parameters) {
    const auto & name = p.get_name();
    if (name == "encoder" &&
      std::set<std::string>{"v4l2h264enc", "x264enc", "openh264enc"}.count(p.as_string()) == 0)
    {
      result.reason = "encoder must be v4l2h264enc, x264enc or openh264enc";
    } else if (name == "width" && hardware && p.as_int() > kHardwareEncoderMaxWidth) {
      result.reason = "the hardware encoder supports at most 1920 pixels of width";
    } else if (name == "height" && hardware && p.as_int() > kHardwareEncoderMaxHeight) {
      result.reason = "the hardware encoder supports at most 1080 pixels of height";
    } else if (name == "h264_profile" &&
      std::set<std::string>{"baseline", "main", "high"}.count(p.as_string()) == 0)
    {
      result.reason = "h264_profile must be baseline, main or high";
    } else if (name == "autofocus_mode" &&
      std::set<std::string>{"continuous", "auto", "manual"}.count(p.as_string()) == 0)
    {
      result.reason = "autofocus_mode must be continuous, auto or manual";
    }
    if (!result.reason.empty()) {
      result.successful = false;
      break;
    }
  }
  return result;
}

void CameraNode::on_parameters_set(const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & p : parameters) {
    if (kRestartParameters.count(p.get_name())) {
      restart_requested_ = true;
      last_stream_change_ = std::chrono::steady_clock::now();
    } else if (kLiveParameters.count(p.get_name())) {
      controls_changed_ = true;
    }
  }
  if (restart_requested_) {
    settings_ = read_settings();
    RCLCPP_INFO(
      get_logger(), "new stream settings: %dx%d @ %d fps, %d bit/s, keyframe every %d frames, %s",
      settings_.width, settings_.height, settings_.framerate, settings_.bitrate,
      settings_.keyframe_interval, settings_.h264_profile.c_str());
  }
}

void CameraNode::on_timer()
{
  if (pipeline_ && poll_bus()) {
    stop_pipeline();
  }

  raw_subscribers_ = raw_pub_->get_subscription_count();
  ffmpeg_subscribers_ = ffmpeg_pub_->get_subscription_count();
  foxglove_subscribers_ = foxglove_pub_->get_subscription_count();
  camera_info_subscribers_ = camera_info_pub_->get_subscription_count();

  const auto now = std::chrono::steady_clock::now();
  const bool wanted = !get_parameter("lazy").as_bool() ||
    raw_subscribers_ + ffmpeg_subscribers_ + foxglove_subscribers_ > 0;
  if (wanted) {
    last_wanted_ = now;
  }

  if (restart_requested_ && now - last_stream_change_ >= kSettleDelay) {
    restart_requested_ = false;
    if (pipeline_) {
      stop_pipeline();
      last_start_attempt_ = {};  // restart right away
    }
  }

  if (!pipeline_) {
    if (wanted && now - last_start_attempt_ >= kRetryDelay) {
      last_start_attempt_ = now;
      start_pipeline();
    }
    return;
  }
  if (!wanted && now - last_wanted_ > kIdleTimeout) {
    RCLCPP_INFO(get_logger(), "no more subscribers: stopping the camera");
    stop_pipeline();
    return;
  }
  update_valves();
  if (second_keyframe_pending_ && now - first_keyframe_request_ >= kSecondKeyframeDelay) {
    second_keyframe_pending_ = false;
    request_keyframe();
  }
  if (controls_changed_) {
    controls_changed_ = false;
    apply_camera_controls();
  }
}

bool CameraNode::start_pipeline()
{
  const auto description = build_pipeline_description(settings_);
  RCLCPP_DEBUG(get_logger(), "pipeline: %s", description.c_str());

  GError * error = nullptr;
  pipeline_ = gst_parse_launch(description.c_str(), &error);
  if (error != nullptr) {
    RCLCPP_ERROR(get_logger(), "cannot build the pipeline: %s", error->message);
    g_error_free(error);
    if (pipeline_) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
    return false;
  }

  source_ = gst_bin_get_by_name(GST_BIN(pipeline_), "source");
  encoded_valve_ = gst_bin_get_by_name(GST_BIN(pipeline_), "encoded_valve");
  raw_valve_ = gst_bin_get_by_name(GST_BIN(pipeline_), "raw_valve");
  encoded_sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "encoded_sink");
  GstElement * raw_sink = gst_bin_get_by_name(GST_BIN(pipeline_), "raw_sink");

  GstAppSinkCallbacks encoded_callbacks{};
  encoded_callbacks.new_sample = &CameraNode::on_encoded_sample;
  gst_app_sink_set_callbacks(GST_APP_SINK(encoded_sink_), &encoded_callbacks, this, nullptr);
  GstAppSinkCallbacks raw_callbacks{};
  raw_callbacks.new_sample = &CameraNode::on_raw_sample;
  gst_app_sink_set_callbacks(GST_APP_SINK(raw_sink), &raw_callbacks, this, nullptr);
  gst_object_unref(raw_sink);

  // Buffer timestamps are then in the realtime clock's frame: base time +
  // PTS is the capture time as the system clock sees it (libcamerasrc maps
  // the sensor timestamp onto the pipeline clock).
  GstClock * clock = gst_system_clock_obtain();
  g_object_set(clock, "clock-type", GST_CLOCK_TYPE_REALTIME, nullptr);
  gst_pipeline_use_clock(GST_PIPELINE(pipeline_), clock);
  gst_object_unref(clock);

  apply_camera_controls();
  encoded_subscribers_seen_ = 0;
  update_valves();

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    poll_bus();
    RCLCPP_ERROR(
      get_logger(), "cannot start the camera (is another process using it? the Pollen "
      "daemon, for instance); retrying");
    stop_pipeline();
    return false;
  }
  RCLCPP_INFO(
    get_logger(), "camera streaming: %dx%d @ %d fps, %s at %d bit/s",
    settings_.width, settings_.height, settings_.framerate, settings_.encoder.c_str(),
    settings_.bitrate);
  return true;
}

void CameraNode::stop_pipeline()
{
  if (!pipeline_) {
    return;
  }
  // blocks until the streaming threads (and so the appsink callbacks) are done
  gst_element_set_state(pipeline_, GST_STATE_NULL);
  for (auto ** element : {&source_, &encoded_valve_, &raw_valve_, &encoded_sink_}) {
    if (*element) {
      gst_object_unref(*element);
      *element = nullptr;
    }
  }
  gst_object_unref(pipeline_);
  pipeline_ = nullptr;
  raw_active_ = false;
}

bool CameraNode::poll_bus()
{
  bool failed = false;
  GstBus * bus = gst_element_get_bus(pipeline_);
  while (GstMessage * msg = gst_bus_pop_filtered(
      bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_WARNING | GST_MESSAGE_EOS)))
  {
    GError * error = nullptr;
    gchar * debug = nullptr;
    switch (GST_MESSAGE_TYPE(msg)) {
      case GST_MESSAGE_ERROR:
        gst_message_parse_error(msg, &error, &debug);
        RCLCPP_ERROR(
          get_logger(), "%s: %s (%s)", GST_OBJECT_NAME(msg->src), error->message,
          debug ? debug : "");
        failed = true;
        break;
      case GST_MESSAGE_WARNING:
        gst_message_parse_warning(msg, &error, &debug);
        RCLCPP_WARN(get_logger(), "%s: %s", GST_OBJECT_NAME(msg->src), error->message);
        break;
      default:
        RCLCPP_ERROR(get_logger(), "the camera stream ended");
        failed = true;
        break;
    }
    if (error) {
      g_error_free(error);
    }
    g_free(debug);
    gst_message_unref(msg);
  }
  gst_object_unref(bus);
  return failed;
}

void CameraNode::update_valves()
{
  const size_t encoded_subscribers = ffmpeg_subscribers_ + foxglove_subscribers_;
  gboolean encoded_dropping = TRUE;
  g_object_get(encoded_valve_, "drop", &encoded_dropping, nullptr);
  if (encoded_subscribers > 0 && encoded_dropping) {
    // the first packets out must be a keyframe, or the decoders start on
    // P-frames whose references they never saw
    wait_for_keyframe_ = true;
    g_object_set(encoded_valve_, "drop", FALSE, nullptr);
  } else if (encoded_subscribers == 0 && !encoded_dropping) {
    g_object_set(encoded_valve_, "drop", TRUE, nullptr);
  }
  if (encoded_subscribers > encoded_subscribers_seen_) {
    // a new subscriber: give it a keyframe instead of waiting for the next one
    request_keyframe();
    first_keyframe_request_ = std::chrono::steady_clock::now();
    second_keyframe_pending_ = true;
  }
  encoded_subscribers_seen_ = encoded_subscribers;

  raw_active_ = raw_subscribers_ > 0;
  g_object_set(raw_valve_, "drop", raw_active_ ? FALSE : TRUE, nullptr);
}

void CameraNode::request_keyframe()
{
  gst_element_send_event(
    encoded_sink_, gst_video_event_new_upstream_force_key_unit(GST_CLOCK_TIME_NONE, TRUE, 0));
}

void CameraNode::apply_camera_controls()
{
  if (settings_.source != "libcamera" || !source_) {
    return;
  }
  const auto af_mode = get_parameter("autofocus_mode").as_string();
  gst_util_set_object_arg(G_OBJECT(source_), "af-mode", af_mode.c_str());
  if (af_mode == "manual") {
    g_object_set(
      source_, "lens-position", static_cast<gfloat>(get_parameter("lens_position").as_double()),
      nullptr);
  }
  g_object_set(
    source_, "exposure-value", static_cast<gfloat>(get_parameter("exposure_value").as_double()),
    nullptr);
}

GstFlowReturn CameraNode::on_encoded_sample(GstAppSink * sink, gpointer self)
{
  GstSample * sample = gst_app_sink_pull_sample(sink);
  if (sample) {
    static_cast<CameraNode *>(self)->handle_encoded(sample);
    gst_sample_unref(sample);
  }
  return GST_FLOW_OK;
}

GstFlowReturn CameraNode::on_raw_sample(GstAppSink * sink, gpointer self)
{
  GstSample * sample = gst_app_sink_pull_sample(sink);
  if (sample) {
    static_cast<CameraNode *>(self)->handle_raw(sample);
    gst_sample_unref(sample);
  }
  return GST_FLOW_OK;
}

builtin_interfaces::msg::Time CameraNode::stamp_of(GstSample * sample)
{
  // PTS -> running time through the segment (encoders shift theirs: x264enc
  // by an hour), then + base time -> the clock's, i.e. the system's, time
  const GstClockTime pts = GST_BUFFER_PTS(gst_sample_get_buffer(sample));
  const GstClockTime running_time = gst_segment_to_running_time(
    gst_sample_get_segment(sample), GST_FORMAT_TIME, pts);
  if (!GST_CLOCK_TIME_IS_VALID(running_time) || get_clock()->ros_time_is_active()) {
    return now();  // simulated time: the capture time means nothing there
  }
  return rclcpp::Time(
    static_cast<int64_t>(gst_element_get_base_time(pipeline_) + running_time), RCL_SYSTEM_TIME);
}

void CameraNode::handle_encoded(GstSample * sample)
{
  GstBuffer * buffer = gst_sample_get_buffer(sample);
  const bool keyframe = !GST_BUFFER_FLAG_IS_SET(buffer, GST_BUFFER_FLAG_DELTA_UNIT);
  if (wait_for_keyframe_) {
    if (!keyframe) {
      return;
    }
    wait_for_keyframe_ = false;
  }

  int width = 0, height = 0;
  const GstStructure * caps = gst_caps_get_structure(gst_sample_get_caps(sample), 0);
  gst_structure_get_int(caps, "width", &width);
  gst_structure_get_int(caps, "height", &height);

  GstMapInfo map;
  if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
    return;
  }
  std_msgs::msg::Header header;
  header.stamp = stamp_of(sample);
  header.frame_id = frame_id_;

  if (ffmpeg_subscribers_ > 0) {
    ffmpeg_image_transport_msgs::msg::FFMPEGPacket msg;
    msg.header = header;
    msg.width = width;
    msg.height = height;
    msg.encoding = "h264";
    // the decoder maps pts back to the header stamp: any unique counter works
    msg.pts = packet_count_++;
    msg.flags = keyframe ? 1 : 0;  // AV_PKT_FLAG_KEY
    msg.is_bigendian = false;
    msg.data.assign(map.data, map.data + map.size);
    ffmpeg_pub_->publish(std::move(msg));
  }
  if (foxglove_subscribers_ > 0) {
    foxglove_msgs::msg::CompressedVideo msg;
    msg.timestamp = header.stamp;
    msg.frame_id = frame_id_;
    msg.format = "h264";
    msg.data.assign(map.data, map.data + map.size);
    foxglove_pub_->publish(std::move(msg));
  }
  gst_buffer_unmap(buffer, &map);

  if (!raw_active_) {
    publish_camera_info(header, width, height);
  }
}

void CameraNode::handle_raw(GstSample * sample)
{
  GstVideoInfo info;
  if (!gst_video_info_from_caps(&info, gst_sample_get_caps(sample))) {
    return;
  }
  GstBuffer * buffer = gst_sample_get_buffer(sample);
  GstVideoFrame frame;
  if (!gst_video_frame_map(&frame, &info, buffer, GST_MAP_READ)) {
    return;
  }

  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header.stamp = stamp_of(sample);
  msg->header.frame_id = frame_id_;
  msg->width = GST_VIDEO_INFO_WIDTH(&info);
  msg->height = GST_VIDEO_INFO_HEIGHT(&info);
  msg->encoding = "yuv422_yuy2";
  msg->is_bigendian = false;
  msg->step = msg->width * 2;
  msg->data.resize(static_cast<size_t>(msg->step) * msg->height);
  const auto * src = static_cast<const uint8_t *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0));
  const size_t stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
  for (uint32_t row = 0; row < msg->height; ++row) {
    std::memcpy(&msg->data[row * msg->step], src + row * stride, msg->step);
  }
  gst_video_frame_unmap(&frame);

  const auto header = msg->header;
  raw_pub_->publish(std::move(msg));
  publish_camera_info(header, GST_VIDEO_INFO_WIDTH(&info), GST_VIDEO_INFO_HEIGHT(&info));
}

void CameraNode::publish_camera_info(
  const std_msgs::msg::Header & header, int width, int height)
{
  if (camera_info_subscribers_ == 0) {
    return;
  }
  auto info = camera_info_manager_->getCameraInfo();
  if (info.width != static_cast<uint32_t>(width) || info.height != static_cast<uint32_t>(height)) {
    const double sx = static_cast<double>(width) / std::max(1u, info.width);
    const double sy = static_cast<double>(height) / std::max(1u, info.height);
    if (info.width > 0 && info.height > 0 && std::abs(sx - sy) < 0.01) {
      // same field of view at another resolution: the intrinsics scale
      for (auto * k : {&info.k[0], &info.k[2], &info.p[0], &info.p[2], &info.p[3]}) {
        *k *= sx;
      }
      for (auto * k : {&info.k[4], &info.k[5], &info.p[5], &info.p[6], &info.p[7]}) {
        *k *= sy;
      }
    } else {
      if (info.width > 0) {
        RCLCPP_WARN_ONCE(
          get_logger(), "the calibration (%ux%u) does not match the image aspect ratio (%dx%d): "
          "publishing an uncalibrated camera_info", info.width, info.height, width, height);
      }
      info = sensor_msgs::msg::CameraInfo();
    }
    info.width = width;
    info.height = height;
  }
  info.header = header;
  camera_info_pub_->publish(info);
}

}  // namespace reachy_mini_camera_driver
