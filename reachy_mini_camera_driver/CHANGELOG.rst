^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package reachy_mini_camera_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2026-09-23)
------------------

* Add reachy_mini_camera_driver: libcamera + hardware H.264 camera node
  A GStreamer pipeline off the ROS thread: libcamerasrc (the ISP scales and
  converts) -> tee -> v4l2h264enc (Raspberry Pi hardware encoder, importing
  libcamera's dmabufs in place) and -> raw YUY2 frames.
  - publishes image_raw/ffmpeg (ffmpeg_image_transport packets), image_raw/h264
  (foxglove CompressedVideo), image_raw (raw, for nodes on the robot) and
  camera_info (camera_info_manager, scaled to the current resolution)
  - dynamic parameters: width, height, framerate, bitrate (the compression
  level), keyframe_interval, h264_profile restart the camera once they have
  settled; autofocus_mode, lens_position, exposure_value apply live
  - each output costs nothing without subscribers (valves); with `lazy` the
  camera stops 5 s after the last one leaves; a new subscriber gets a
  keyframe at once (and a second one 0.5 s later), SPS/PPS repeated
  - capture timestamps (libcamera sensor time, in system time)
  - source:=test / v4l2:<dev> and encoder:=x264enc / openh264enc to develop
  without the robot
  On the CM4: 8 % of a core at 720p30, 10 % at 1080p30 (37 % when the
  encoder copies the frames instead of importing them).
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

0.4.0 (2026-09-17)
------------------

0.3.0 (2026-09-16 18:02)
------------------------

0.2.0 (2026-09-16 17:46)
------------------------
