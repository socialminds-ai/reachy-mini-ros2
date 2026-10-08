^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package reachy_mini_camera_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.3.0 (2026-10-07)
------------------

1.2.2 (2026-10-07)
------------------
* camera_driver: drop ffmpeg_image_transport and the GStreamer headers from the runtime dependencies
  The node publishes its H.264 packets with a plain publisher
  (ffmpeg_image_transport_msgs): the ffmpeg_image_transport plugin only
  decodes them, on the subscribers' side. As an exec_depend, it brought
  cv_bridge and all of OpenCV (VTK, OpenMPI, GDAL, Qt5: ~950 MB) into the
  robot image. The GStreamer -dev packages become build-only (their
  runtime libraries come from shlibs), and gstreamer1.0-plugins-base
  (appsink, videoconvert, videoscale) an explicit exec_depend.
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

1.2.1 (2026-10-05)
------------------

1.2.0 (2026-10-05)
------------------

1.1.0 (2026-10-05)
------------------
* Camera driver: shipped calibrations, chosen by sensor mode
  calibration_wide.yaml and calibration_narrow.yaml (IMX708 wide, 1280x720,
  focus at 1.5 m), measured on reachy-mini-1 with a 9x7 / 25 mm checkerboard:
  - wide: rational model (plumb_bob could not reach the image corners), 136
  views, RMS 0.507 px, 102.8 x 67.4 deg;
  - narrow: plumb_bob, 104 views, 76.0 x 47.7 deg.
  With an empty camera_info_url (the default) and libcamera, the node loads
  the file of the sensor mode actually in use: wide when field_of_view is
  wide or the width is 1920 or more (libcamera then uses the full sensor),
  narrow otherwise. It reloads it when the camera restarts with another mode;
  other 16:9 sizes are scaled as before. A camera_info_url still overrides.
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Camera driver: field_of_view parameter (wide or narrow sensor mode)
  libcamera picked the IMX708 sensor mode from the output size: 1280x720 came
  from the 1536x864 central crop (about 77 deg), 1920x1080 from the full
  sensor (about 102 deg), so the field of view, and the calibration, changed
  with the resolution. field_of_view pins it: 'narrow' keeps the crop (width
  up to 1536), 'wide' (the default) asks the ISP for a second, discarded
  1920x1080 stream, which makes libcamera use the full sensor. libcamerasrc
  0.7 can neither select the mode nor output the packed raw stream that would
  set it.
  Tested on the robot at 1280x720: both modes stream at 30 fps (camera_node
  about 14 % of a core in wide, 11 % in narrow).
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Camera driver: set_camera_info in the camera's namespace
  camera_info_manager names its service ns + "/set_camera_info", and the
  namespace defaults to empty: the service was /set_camera_info, where
  camera_calibration does not find it. It is now /camera/set_camera_info,
  next to camera_info.
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Camera driver: printable calibration checkerboard
  A4 landscape, 9x7 squares of 25 mm (8x6 inner corners), with its size,
  the matching cameracalibrator arguments and a 100 mm bar to check the
  print scale.
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Camera driver: fixed focus at 1.5 m by default
  autofocus_mode now defaults to manual, with lens_position at 0.667 dioptre
  (1.5 m): the focal length moves with the lens, so a fixed focus keeps a
  camera calibration valid. Both stay changeable at runtime.
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

1.0.0 (2026-10-02)
------------------

0.8.0 (2026-10-02)
------------------
* update changelogs
* Contributors: Séverin Lemaignan

0.7.0 (2026-10-01)
------------------

0.6.0 (2026-09-23)
------------------

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
