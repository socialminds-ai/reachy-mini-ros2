# reachy_mini_description

URDF and meshes of the Reachy Mini, generated from the Pollen Robotics SDK model
(`reachy_mini/descriptions/reachy_mini/urdf`, Apache-2.0) by
`scripts/convert_sdk_urdf.py`. Regenerate after an SDK update with:

```sh
python3 scripts/convert_sdk_urdf.py <site-packages>/reachy_mini/descriptions/reachy_mini/urdf .
```

The Stewart platform is a closed kinematic chain, which `robot_state_publisher`
cannot solve (the passive joint angles are never measured). The model is
therefore restructured: `head` and the six rods `rod_1..6` hang under `base_link`
through joints (`head_pose`, `rod_i_pose`) that never appear in `joint_states`, so
`robot_state_publisher` never publishes them and the driver broadcasts their
transforms from its forward kinematics instead (a URDF `floating` joint would
not do: kdl_parser turns it into a fixed joint). `robot_state_publisher` publishes everything else from
`joint_states` (`body_yaw`, `stewart_1..6`, `right_antenna`, `left_antenna`).

```sh
ros2 launch reachy_mini_description description.launch.py rviz:=true
```

Frames of interest: `base_link` (robot base), `head` (the head pose the driver
and the SDK command), `camera` and `camera_optical`.
