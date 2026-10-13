# Design of the Reachy Mini bare-metal driver

This document explains why the driver exists, what it borrows from Pollen Robotics'
daemon and SDK, and how those borrowings shaped the ROS 2 packages of this repository.
It is written for someone who wants to modify the driver. The user-facing documentation
is in `README.md` (installation, usage, tests) and `REFERENCE.md` (packages, parameters,
topics); this document is about the reasons.

## 1. Why a bare-metal driver

The robot ships with the Pollen daemon (`reachy-mini-daemon`): a Python process that owns
the servo bus, runs the control loop, and exposes the robot over HTTP (port 8000),
WebSocket and zenoh. A ROS integration on top of it would be a bridge: one more process,
one more middleware, one more serialisation hop between a ROS message and a servo goal.

The driver in this repository is the control loop of the daemon, rewritten as a ROS 2
node that opens `/dev/ttyAMA3` itself: one process fewer on the Raspberry Pi, ROS as the
only middleware between a behaviour node and the servos, and a target published on a
topic reaches the motors on the next 20 ms tick with nothing in between but the node's
own timer. The robot's whole behaviour is visible through ROS tools (`/joint_states`,
`/tf`, the ROS4HRI skills).

What is lost by stopping the daemon: its sound output (the emotions library ships one
sound per move; only the daemon plays it), the camera and its WebRTC stream, the HTTP API,
and the Pollen apps that use them. The serial port has exactly one owner, so the daemon
and the driver are mutually exclusive: the systemd unit declares
`Conflicts=reachy-mini-daemon.service`, and the README tells the user to stop the daemon
before running the driver by hand.

## 2. What the Pollen daemon does every 20 ms

The daemon is Python on top of two Rust crates that Pollen publishes as Python wheels.
The same two crates are what the driver reuses, so the way the daemon uses them is the
reference for the driver. Excerpts below are from the `reachy-mini` package, version
1.10.0.

### The loop

`RobotBackend.run()` starts with a cold forward kinematics (20 iterations) so that the
first target is the pose the head is actually in, then loops at
`control_loop_frequency = 50.0` Hz:

```python
# reachy_mini/daemon/backend/robot/backend.py, RobotBackend.run
period = 1.0 / self.control_loop_frequency
head_positions, _ = self.get_all_joint_positions()
self.current_head_pose = self.head_kinematics.fk(np.array(head_positions), no_iterations=20)

while not self.should_stop.is_set():
    start_t = time.time()
    self._update()
    took = time.time() - start_t
    sleep_time = period - took
    if sleep_time < 0:
        self.logger.debug(f"Control loop took too long: {took * 1000:.3f} ms, ...")
        sleep_time = 0.001
    next_call_event.clear()
    next_call_event.wait(sleep_time)
```

There is no catch-up: a late tick is simply late. `_update()` does, in this order:

1. Write the goals computed on the previous tick, if torque is on:

   ```python
   # reachy_mini/daemon/backend/robot/backend.py, RobotBackend._update
   if self._torque_enabled:
       if self._current_head_operation_mode != 0:  # position control
           if self.target_head_joint_positions is not None:
               self.c.set_stewart_platform_position(self.target_head_joint_positions[1:].tolist())
               self.c.set_body_rotation(self.target_head_joint_positions[0])
       if self._current_antennas_operation_mode != 0:
           if self.target_antenna_joint_positions is not None:
               self.c.set_antennas_positions(self.target_antenna_joint_positions.tolist())
   ```

2. Read the nine positions and run FK (3 iterations, warm-started) to refresh the
   present head pose:

   ```python
   head_positions, antenna_positions = self.get_all_joint_positions()
   self.update_head_kinematics_model(np.array(head_positions), np.array(antenna_positions))
   ```

3. Run IK only if a new target arrived (`set_target_head_pose()` sets `ik_required`):

   ```python
   if self.ik_required:
       try:
           self.update_target_head_joints_from_ik(self.target_head_pose, self.target_body_yaw)
       except ValueError as e:
           log_throttling.by_time(self.logger, interval=0.5).warning(f"IK error: {e}")
   ```

   An unreachable pose is an exception, not a NaN that reaches the servos:

   ```python
   # reachy_mini/daemon/backend/abstract.py, update_target_head_joints_from_ik
   joints = self.head_kinematics.ik(pose, body_yaw=body_yaw)
   if joints is None or np.any(np.isnan(joints)):
       raise ValueError("WARNING: Collision detected or head pose not achievable!")
   self.target_head_joint_positions = joints
   ```

4. Publish joint positions and head pose (zenoh, WebSocket), and once a second read the
   servos' hardware error registers. The daemon also keeps loop statistics and reboots
   servos that report an error; the driver does neither (see section 7).

Playing a move is not in the loop. `play_move()` in `abstract.py` is a coroutine that
evaluates the move at up to 100 Hz and sets targets; the loop picks them up:

```python
# reachy_mini/daemon/backend/abstract.py, play_move
if initial_goto_duration > 0.0:
    start_head_pose, start_antennas_positions, start_body_yaw = move.evaluate(0.0)
    await self.goto_target(head=start_head_pose, antennas=start_antennas_positions,
                           duration=initial_goto_duration, body_yaw=start_body_yaw)
t0 = time.time()
while time.time() - t0 < move.duration:
    t = time.time() - t0
    head, antennas, body_yaw = move.evaluate(t)
    if head is not None:
        self.set_target_head_pose(head)
    if body_yaw is not None:
        self.set_target_body_yaw(body_yaw)
    if antennas is not None:
        self.set_target_antenna_joint_positions(antennas)
    ...
```

Two things the driver copies: the initial goto (reach the first pose smoothly so a move
never starts with a jump), and the split between the thing that decides the target and
the loop that writes it.

### The two Rust crates

`reachy_mini_motor_controller` (`ReachyMiniMotorController::new(serial_port)`) opens the
port at 1 Mbaud and talks Dynamixel protocol 2 (through `rustypot`'s XL330 register
table) to nine XL330/XC330 servos: ID 10 (body yaw), 11 to 16 (Stewart platform), 17
and 18 (right and left antenna). `read_all_positions()` is one sync read of the nine
present positions, `set_all_goal_positions([f64; 9])` one sync write of the nine goals,
`enable_torque()` / `disable_torque()` act on all nine. Angles are radians, always in the
order `[body_yaw, stewart_1..6, right_antenna, left_antenna]`.

`reachy_mini_rust_kinematics` solves the Stewart platform (six motors, six rods of
0.085 m on 0.04 m horns, one moving plate):

* IK is analytical: `inverse_kinematics(pose, body_yaw)` gives the six motor angles in
  closed form, or NaN if the pose is not reachable; `inverse_kinematics_safe` adds the
  body-yaw logic of section 3 and returns seven joints.
* FK is iterative: `forward_kinematics(joints, body_yaw)` is a Newton-style step
  warm-started from the previous pose (`reset_forward_kinematics` sets the warm start).
  The daemon runs 3 steps per tick and 20 on a cold start; so does the driver.

## 3. Conventions inherited from the SDK

The driver keeps the SDK's conventions unchanged, so that move files, poses and joint
arrays mean the same thing on both sides.

**Head pose.** A 4x4 homogeneous matrix of the `head` frame in the robot base frame,
identity at rest (head upright, centred). The platform is `head_z_offset = 0.177` m
above the base (`kinematics_data.json`); the SDK's `AnalyticalKinematics` adds it before
calling the crate and subtracts it after, and `rmc_kinematics_ik`/`rmc_kinematics_fk` do
the same, so users never see it. `~/target_head_pose`, `~/head_pose` and the move files
all use this convention; only the TF frame `base_link -> head` carries the real height.

**Automatic body yaw.** The SDK's default (`automatic_body_yaw=True`) calls
`inverse_kinematics_safe` with two limits: the head may not yaw more than 65 degrees
relative to the body, and the body may not turn more than 160 degrees. If a target asks
for more relative yaw, the solver turns the body. This is why IK returns seven joints,
body yaw first, and why `~/target_body_yaw` is a request the solver may override.
`rmc_kinematics_ik` hard-codes the same limits; `rmc_kinematics_ik_fixed_yaw` is the
non-automatic variant, kept for the SDK comparison tool.

**Joint order.** `[body_yaw, stewart_1..6, right_antenna, left_antenna]` is the motor
order of the crate, the `joint_states` order of the driver (`MOTOR_NAMES` in `bus.py`,
`kMotorNames` in `bus.cpp`), and the joint names of the URDF.

**Recorded moves.** A move is a JSON file with `time` (sample times at 50 Hz) and
`set_target_data` (one entry per sample: `head` 4x4, `antennas` `[right, left]`,
`body_yaw`), plus a `description`:

```text
{"description": "test move: a 1 s nod",
 "time": [0.0, 0.02, 0.04, ...],
 "set_target_data": [
   {"head": [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
    "antennas": [-0.0, 0.0], "body_yaw": 0.0, "check_collision": false},
   ...]}
```

Evaluation at time `t` finds the two surrounding samples by bisection and interpolates:
lerp for antennas and body yaw, translation lerp + rotation slerp for the pose
(`linear_pose_interpolation` in the SDK, `interpolate()` in `kinematics.cpp`). The
Python driver delegates to the SDK's `RecordedMove`; `moves.cpp` reimplements
`RecordedMove::fromFile` and `evaluate` (`std::upper_bound`, clamped to `[0, duration]`,
missing fields default to identity/zeros).

**The emotions library and the HuggingFace cache.** Pollen's moves live in the dataset
`pollen-robotics/reachy-mini-emotions-library` (about 85 moves, plus one sound file each).
The `reachy_mini_expressions_data` package ships a pinned copy (with the dances library
and the SDK's sounds), and the driver uses it by default: no network, no cache, the
moves versioned with the stack. `MoveLibrary` (`moves.py`, `moves.cpp`) takes
`package://<package>/<directory>`, resolved through the ament index
(`get_package_share_path`), a local directory of `<name>.json`, or a dataset name.
For a name, the Python version calls the SDK's `RecordedMoves(dataset)`,
which downloads through `huggingface_hub`; the C++ version has no network code and
resolves the cache layout itself (`MoveLibrary::resolveDataset`): under `$HF_HOME` or
`~/.cache/huggingface`, `hub/datasets--<org>--<name>/refs/main` gives the snapshot SHA
and `snapshots/<sha>/<move>.json` (or `snapshots/<sha>/data/<move>.json` in newer
datasets) is the file. The dataset has to be downloaded once, for instance with the SDK.

## 4. How that shaped the ROS design

### Driver and skills

The driver does what the daemon's loop does and nothing more: it owns the bus, keeps the
head where it was told, plays recorded moves, and reports state. Everything a robot
*does* (looking at someone, reacting, sequencing moves) is a separate node that publishes
targets or calls the action. This mirrors the daemon's own split between the loop and
`play_move()`/`goto_target()`, and keeps the only process that can hurt the hardware
small.

### The bus behind an interface

Everything that touches hardware goes through `MotorBus` (`bus.py`, `bus.hpp`):
`read_positions()`, `write_goal_positions(goals)`, `set_torque(enabled)`
(`readPositions`, `writeGoalPositions`, `setTorque` in C++). `RealBus` wraps the crate,
through `ReachyMiniMotorController` in Python and `rmc_motors_*` in C++. `FakeBus` is an
ideal servo: what you write is what you read on the next tick, provided torque is on
(like a real servo), and it counts writes. `serial_port:=fake` selects it; the tests
construct `DriverNode(bus=FakeBus())` directly. The whole node, action server included,
runs unchanged on a laptop without the robot and in `colcon test`.

### The 50 Hz tick

The loop is one timer callback, `DriverNode._tick()` / `DriverNode::tick()`, at the
`rate` parameter (50 Hz). It mirrors the daemon's `_update()` with one deliberate
change: it reads, computes and writes within the same tick, instead of writing the
previous tick's goals first. The FK and the goals of a tick then belong to the same
sample, which is simpler to test. In order:

1. `bus.read_positions()`; on failure, log (throttled to 1 s) and skip the tick.
2. Under the lock: store the positions, `kinematics.fk(positions[:7], 3 iterations)`.
3. Decide the target, under the same lock:
   * if a move is playing, `move.evaluate(t)` at `t = now - t0`, clamped just below
     `duration`; when `t >= duration` or the move was cancelled, clear it and signal the
     action thread (`_move_done` event, `move_done_` atomic);
   * else if a goto is in progress, interpolate start to end with `a = min(1, (now -
     t0) / duration)` and clear it when `a >= 1`;
   * else keep the last targets set by the topic callbacks.
4. (C++) The speed limiter (`MotionLimiter`, see *Speed limits* below) moves the
   commanded pose, antennas and body yaw toward those targets by what the limits allow
   in one tick.
5. `_compute_goals()` / `computeGoals()`: `kinematics.ik(pose, body_yaw)` on the
   commanded pose (the targets in Python) gives seven joints; append the two antennas. An unreachable pose gives `None`
   / `std::nullopt` and a throttled warning; nothing is written that tick, so the robot
   holds its last goal.
6. Publish `joint_states` (nine names, nine positions), `~/head_pose` (a `PoseStamped` in
   `base_frame`), and the seven TF transforms of section 5.
7. `bus.write_goal_positions(goals)`, again with a throttled warning on failure.

IK runs every tick, not only when a target changed as in the daemon: the analytical
solver is cheap and this removes a flag. Constructors do the daemon's cold start: read
once, `fk(..., 20 iterations)`, and initialise the target pose, antennas and body yaw
from what was read, so the first goals written are where the head already is.

### Motions as ROS4HRI skills, and the initial goto

The daemon's `play_move()` became the standard `replay_motion` skill
(`motions_skills/action/ReplayMotion`, from https://ros4hri.github.io/skills/) rather
than a private action, so that a behaviour node written for any ROS4HRI robot drives
this one: the goal is `name` + a `std_skills/Meta` (`caller`, `priority`), the result a
`std_skills/Result` error code, the feedback `data_float` = progress 0..1. An action fits
because a move takes seconds, can fail (unknown move, missing dataset), reports progress
and can be cancelled. The sibling skills share the machinery: `list_motions` (a service)
returns what `replay_motion` accepts; `execute_cartesian_trajectory` plays a
`moveit_msgs/CartesianTrajectory` of head poses; `execute_joint_trajectory` plays a
`trajectory_msgs/JointTrajectory` straight on the motors, bypassing IK (with `safe_mode`
refusing the six platform joints, which are meaningless one at a time). All four are one
`Motion` interface in C++ (`trajectories.hpp`: `duration()` and `evaluate(t)` returning
optional head pose, antennas, body yaw or raw joint goals), so the loop does not know
which skill it is playing.

The goal callback rejects a motion whose `meta.priority` is lower than the running one;
otherwise the execute callback runs in its own thread (a `ReentrantCallbackGroup` on a
3-thread `MultiThreadedExecutor` in Python; a detached `std::thread` from the accepted
callback in C++). That thread never touches the bus:

1. look the motion up (`MoveLibrary.get`, in `dataset` then `extra_datasets`); a
   loading error aborts the goal with `ROS_EBADMSG` and the available names in
   `error_msg`, as the skill spec asks;
2. take the single motion slot: the motion that was playing is flagged *preempted* and
   its thread returns `ROS_EINTR`;
3. if `motion_initial_goto > 0` and the motion has a head pose,
   `goto(first_pose, first_antennas, duration)` and a sleep of that length, so the loop
   glides to the first sample (in C++ the goto also brings the body yaw, and lasts
   longer than `motion_initial_goto` when the distance needs it, see *Speed limits*) (the slot is reserved, but not *started*, during that
   glide: the loop keeps running the goto);
4. start the clock, then only wait: every 50-100 ms publish feedback and check for a
   cancel request; on cancel set the flag, wait for the loop to acknowledge, return
   `ROS_ECANCELED`; when the loop signals the end, return `ROS_ENOERR`.

A cancelled move leaves the robot where it is. A joint-space motion ends by resetting
the Cartesian targets to where the joints actually are, so the next tick's IK does not
snap the head back. The same `Goto` mechanism (`Goto` in `driver_node.py`,
`DriverNode::Goto` in C++) serves the wake-up and sleep sequences at startup and exit;
the `wake_up` and `go_to_sleep` skills run their gotos as motions instead (see below).

### look_at

`interaction_skills/action/LookAt` is the other skill the hardware can honour without
perception: the C++ driver keeps a *gaze* state next to the motion slot. With an empty
`policy` the goal succeeds at once and the head looks at `target` (a `PointStamped` in
any TF frame) until another goal or a reset replaces it. The target is transformed into
`base_frame` *once*, when the goal is received, and that point stays fixed: a target
expressed in a frame that moves with the gaze (`head`, `camera`, or the body, which the
automatic body yaw of the IK turns too) would otherwise run away from the head as it
turns, and the head would chase it. `GLANCE` does the same for `glance_duration`
seconds and then restores whatever gaze was in force before (or the rest pose).
`TRACKING` follows a moving frame (a person, an object): the loop transforms the target
into `base_frame` every tick through a `tf2` buffer, and the goal runs until it is
cancelled, replaced by another gaze command (`ROS_EINTR`), or the target is lost (no
transform for `tracking_lost_timeout` seconds, `ROS_EOTHER`). Tracking a point in one of
the robot's own frames (the URDF links from `/robot_description`, plus `head` and the
rods) other than `base_frame` cannot settle, for the reason above, and logs a loud
warning. `RESET` is a goto to the rest pose. The head
orientation is yaw = atan2(y, x), pitch = -atan2(z, distance), both clamped to what the
platform reaches (about 35 degrees), with the head origin kept at rest height; the
policies that need a perception stack (`RANDOM`, `SOCIAL`, `AUTO`) answer `ROS_ENOTSUP`.

### Targets as topics

`~/target_head_pose` (`geometry_msgs/PoseStamped`), `~/target_antennas`
(`reachy_mini_msgs/AntennasTarget`, `right`/`left` in radians, 0 = up) and
`~/target_body_yaw` (`std_msgs/Float64`) are latest-value inputs: the callback stores the
value under the lock, the next tick uses it. A new head pose cancels a goto in progress;
it does not interrupt a move, since the loop overwrites the targets from the move every
tick until it ends. Topics rather than services because a tracking node publishes
continuously and wants no round trip per sample, like `ReachyMini.set_target()` in the
SDK.

### Speed limits

The servos follow their goal positions as fast as they can, so any discontinuity in the
targets becomes a jerk of the head: a `look_at` target or a topic sample far from the
current pose, the fixed 0.5 s glide from the sleep pose to a move's first sample, a
joint trajectory whose first point is not where the joints are, or the few recorded
moves whose samples jump within milliseconds (`wake-mini-up` has steps of 10 to 20
degrees 1 to 17 ms apart). The C++ driver therefore puts a limiter between the targets
and the IK (`limits.hpp`, in the hardware-independent library and unit-tested there):

* the commanded head pose moves toward the target pose in a straight line and turns
  about the axis of the remaining rotation; antennas and body yaw move independently;
* each of these quantities has a speed limit and an acceleration limit, and slows down
  so as to stop on the target (a bang-bang profile on the speed: `v = min(v_max,
  v_prev + a dt, sqrt(2 a d))` for a remaining distance `d`); the target may move at
  every tick, which is what `look_at` tracking needs;
* two sets of limits: *smooth* ones (`max_speed.*`, `max_acceleration.*`) when the
  target is reactive (gaze, topics), and *safety* caps (`safety_max_speed.*`, speed
  only) while a motion plays, so that authored motions keep their timing. The caps were
  chosen from the emotions library: 99.9 % of its head rotation speeds are under 8 rad/s,
  and the regularly sampled fast moves (`dying1`) stay under 10 rad/s;
* gotos are planned within the smooth limits rather than limited: their duration is at
  least 1.5 times distance over speed (the peak speed of the smoothstep easing they now
  use), and the loop applies only the safety caps during them;
* joint trajectories bypass the limiter (they write joint goals), so they start from the
  current goals, and their antenna and body-yaw joints are clamped to the safety caps.

The Python driver has none of this yet.

### Wake-up, sleep, and why SIGINT is the stop signal

At construction, after `set_torque(True)`, `wake_up_on_start` triggers a 2 s goto to
`INIT_HEAD_POSE` (identity) and `INIT_ANTENNAS` (`[-0.1745, 0.1745]`, about 10 degrees
outwards, which shakes less than vertical). The constants `SLEEP_HEAD_POSE` (head tilted
forward and down) and `SLEEP_ANTENNAS` (`[-3.05, 3.05]`) are the daemon's rest poses.

`shutdown()` cancels any move, then if `sleep_on_shutdown`: a 2 s goto to the sleep pose,
driven by calling `_tick()` by hand every 20 ms until the goto is done or 2.5 s have
passed (the executor may already be gone), then `set_torque(False)`. The head rests on
its mechanical stop, nothing is left powered against gravity, and the daemon can take
over cleanly.

Wake-up and sleep are also skills, for a behaviour that puts the robot to sleep and
wakes it up while the driver keeps running: `wake_up` (`motions_skills/ReplayMotion`)
and `go_to_sleep` (`std_skills/Empty`). They reuse standard interfaces rather than adding
Reachy-specific ones: `go_to_sleep` takes nothing but the skill metadata, and `wake_up`
takes an optional motion name, which is exactly a `ReplayMotion` goal (an empty `name`
means the default wake-up, a goto). Both are actions rather than services: they last
from 2 s to the 15 s of `wake-mini-up`, and can be cancelled or preempted.

They run in the motion slot rather than as plain gotos, so priorities, preemption,
cancel and feedback work as for any motion, and nothing can silently override them: a
plain goto is dropped by the next motion, and would let a `look_at` gaze take the head
back once over (the gaze is cleared when they start). The goto is therefore turned into
a move (`gotoMove` in C++, `GotoMove` in Python): from the commanded pose to the target,
sampled along the gotos' easing (C++), at least 2 s, longer if the smooth limits need
it. `wake_up` turns the torque on first (the motors may have been released), plays the
animation if one is named (with the usual glide to its first sample: `wake-mini-up`
starts from the sleep pose), then the goto to the rest pose, so it always ends in the
same pose whatever the animation's last sample. `go_to_sleep` leaves the torque on:
releasing it is for the end of the process, when another program (the daemon) takes the
servos over; a running driver keeps holding the head.

The exit sequence needs a live ROS context and a few seconds, which dictates how the
process is stopped. In Python, `rclpy`'s default SIGINT handler tears the context down at once,
so `main()` initialises with `SignalHandlerOptions.NO`, installs a SIGINT/SIGTERM handler
that only sets an event, spins with `spin_once` until it is set, then runs
`node.shutdown()` before `rclpy.shutdown()`. In C++, `main.cpp` lets `rclcpp`'s default
handler end `executor.spin()`, then calls `node->shutdown()` before `rclcpp::shutdown()`.
The systemd unit `reachy-mini-driver.service` therefore sets `KillSignal=SIGINT`
(forwarded by `ros2 launch` to the node) and `TimeoutStopSec=10`.

### The same design in Python and C++

Both nodes are named `reachy_mini_driver`, executable `driver_node`, with the same
parameters, topics, action and TF output; `reachy_mini/launch/driver.launch.py` selects
one with `lang:=cpp|python`. The Python driver imports the SDK's wheels
(`reachy_mini_motor_controller`, `reachy_mini.kinematics.analytical_kinematics`,
`reachy_mini.motion.recorded_move`) and reads `reachy_mini/assets/kinematics_data.json`
through `importlib.resources` (`geometry.py`). It is the shortest path to a working
driver, but the `reachy-mini` pip package must be importable by the interpreter that runs
`rclpy`, which on Ubuntu 26.04 means installing it by hand (REFERENCE.md); the `reachy_mini`
metapackage depends only on the C++ driver.

The C++ driver cannot use Python wheels, hence `reachy_mini_c`: a C ABI implemented in
Rust (`rust/reachy_mini_c/src/lib.rs`, `crate-type = ["cdylib", "staticlib"]`) over the
two Pollen crates. Functions return 0 or -1 with a thread-local `rmc_last_error()`;
handles are opaque (`RmcMotors`, `RmcKinematics`); poses are row-major 16-double arrays.
`rmc_kinematics_new()` embeds `kinematics_data.json` (from `reachy_mini` 1.10.0) with
`include_str!` and builds the crate's `Kinematics` from it (`add_branch` per motor, with
`T_motor_world` inverted and the `solution` flag turned into +1/-1), so the C++ driver
needs neither the SDK nor any file at run time.

The crates are vendored (`rust/vendor/`, `VENDORING.md`) rather than fetched: Pollen
publishes them as PyPI source distributions with pyo3 bindings, not on crates.io, and the
build must work offline in `colcon build` and in a Debian package build. Vendoring keeps
`controller.rs` and the `Kinematics` struct verbatim and drops the pyo3/tokio parts; the
comparison tool of section 6 guards against drift.

CMake drives cargo: `reachy_mini_c/CMakeLists.txt` has an `add_custom_command` whose
output is `${CMAKE_CURRENT_BINARY_DIR}/cargo/release/libreachy_mini_c.so`, runs `cargo
build --release --manifest-path rust/Cargo.toml` with `CARGO_TARGET_DIR` in the build
tree, and depends on every `.rs`/`.toml`/`.json` under `rust/`. The result is an imported
target `reachy_mini_c` installed to `lib/`; since imported targets cannot go through
`ament_export_libraries()`, `cmake/reachy_mini_c-extras.cmake` defines
`reachy_mini_c::reachy_mini_c` for downstream packages. `reachy_mini_driver_cpp` wraps it
in `Kinematics` (`kinematics.hpp`: Eigen `Isometry3d` poses, `std::optional` for
unreachable IK) and `RealBus`, and parses move files with `nlohmann_json`.

## 5. TF and the robot model

The Stewart platform is a closed kinematic chain. `robot_state_publisher` walks a tree
from `joint_states`; it can place the body (`body_yaw`), the six motor horns
(`stewart_1..6`) and the antennas (`right_antenna`, `left_antenna`), but the six passive
ball joints between horns, rods and platform are never measured, so it cannot place the
head. The driver has the head pose from its FK anyway, so the split is: the model
publishes what follows from a measured angle, the driver broadcasts the rest.

What the driver broadcasts on `/tf` at the loop rate (`publish_tf`, `base_frame`
parameters; `PlatformGeometry` in `geometry.py`, `Kinematics::headInBase`/`rodsInBase`
in C++):

* `base_link -> head`: the FK pose with `head_z_offset` added back to the translation
  (`head_in_base`).
* `base_link -> rod_1..6`: for leg `i` with motor angle `theta`, the horn tip is
  `T_base_motor_i * arm_length * (cos theta, sin theta, 0)` and the rod's anchor on the
  platform is `T_base_head * branch_position_i`. The rod frame has its origin at the horn
  tip and its x axis along the tip-to-anchor vector, by the minimal rotation from +x
  (`Rotation.align_vectors`, `Quaterniond::FromTwoVectors`); rods have ball joints at
  both ends, so the roll about their axis is arbitrary. The mesh is centred at
  `x = rod_length / 2`. The geometry (`T_motor_world`, inverted to get `T_base_motor`;
  `branch_position`; `motor_arm_length` 0.04 m; `rod_length` 0.085 m) is the same
  `kinematics_data.json` the IK uses, read from the SDK package in Python and through
  `rmc_kinematics_lengths` / `rmc_kinematics_leg` in C++.

The URDF (`reachy_mini_description/urdf/reachy_mini.urdf`) is generated from the SDK's
Onshape export by `scripts/convert_sdk_urdf.py`. The SDK model reaches the head through
one leg and closes the other loops with dummy frames, which suits a solver but not
`robot_state_publisher`. The script drops the passive chains, rods and closing frames,
adds a root `base_link` (identity to the SDK root, fixed joint `base_to_foot`), renames
`yaw_body` to `body_yaw` to match `joint_states`, makes `head` the root of the head
subtree (the platform link `xl_330` re-attached under it through the inverse of the old
`head_frame` origin, so `camera` and `camera_optical` stay on the head), adds six `rod_i`
links with the rod mesh, and strips collision geometry.

The trick is how `head` and `rod_i` hang under `base_link`: through `continuous` joints
`head_pose` and `rod_i_pose` that never appear in `joint_states`.
`robot_state_publisher` publishes a moving joint only once it has received a value for
it, so it leaves those alone and the driver's broadcast fills the gap. A URDF `floating`
joint would be the honest modelling, but `kdl_parser` turns floating joints into fixed
ones, and `robot_state_publisher` would publish a static identity transform that fights
the driver's.

## 6. Testing strategy

Nothing in the tests needs hardware, the SDK's network access, or a robot-specific
environment; `colcon test` runs them all in a `ros:lyrical` container.

* **Driver on the fake bus.** `reachy_mini_driver/test/test_driver.py` (pytest) and
  `reachy_mini_driver_cpp/test/test_driver.cpp` (gtest) build `DriverNode(bus=FakeBus())`
  on a 3-thread executor and check the same things: torque on and the loop writing; the
  wake-up goto reaching `INIT_ANTENNAS` with body yaw 0; nine `/joint_states` positions
  starting with `body_yaw`; a `~/target_head_pose` 2 cm up moving the Stewart angles by
  more than 0.05 rad; `/tf` carrying exactly `head` and `rod_1..6` under `base_link`, head
  at rest height; `replay_motion` on `test/moves/nod.json` (a 1 s nod, antennas to
  -0.3/+0.3 rad at 0.5 s) accepted, wiggling the antennas, sending feedback and
  returning `ROS_ENOERR`; an unknown name answered with `ROS_EBADMSG` and the available
  names; `list_motions` returning `nod`; `go_to_sleep` reaching `SLEEP_ANTENNAS` with the
  torque still on, then `wake_up` turning the torque back on and reaching the rest pose,
  with no name (a goto) and with `nod` as the animation (played, then the rest pose),
  and an unknown animation answered with `ROS_EBADMSG`. The C++ tests add the priority rules (a lower
  priority goal rejected, an equal one preempting with `ROS_EINTR` for the loser), a
  Cartesian and a joint trajectory moving the fake servos (and `safe_mode` refusing the
  platform joints), and `look_at` looking, glancing back, resetting, settling on a target given in the
  head frame, and tracking until cancelled, replaced or lost. `FakeBus` makes
  writes observable and instant, so tests assert on `bus.positions`, not on timing.
* **Moves and kinematics without ROS.** `test_moves.cpp` checks `RecordedMove` loading,
  interpolation and clamping, HuggingFace cache resolution against a fake `$HF_HOME`
  tree, the directory-based library, IK/FK round trips and `interpolate`.
* **Rod-length consistency.** `test_geometry.py` and `Kinematics.RodsHaveTheRodLength`
  compute IK for a rest pose and a moved pose (roll, pitch, yaw, translation), then check
  that every horn tip to anchor distance from `rod_lengths()` equals `rod_length` to
  1e-4 m and that the rod frames are proper rotations. This ties the TF geometry to the
  analytical kinematics: a wrong frame or a wrong inversion of `T_motor_world` would give
  rods of the wrong length.
* **The C shim against the SDK.** `reachy_mini_c/test/test_kinematics.cpp` checks
  `rmc_kinematics_*` against values taken from the SDK (rest pose gives alternating
  +/-0.6265 rad, body yaw passed through, 1 cm up round-trips through FK, 50 cm up is an
  error, NULL handles are errors). `tools/check_reachy_mini_c_against_sdk.py` loads
  `libreachy_mini_c.so` with `ctypes`, draws 200 random poses and body yaws, and compares
  `rmc_kinematics_ik`, `rmc_kinematics_ik_fixed_yaw` and the FK round trip with
  `AnalyticalKinematics`; it passes when IK agrees to 1e-9 rad (in practice 1e-13) and
  unreachable poses agree. Run it after every re-vendoring.

## 7. Known limitations and ideas

* **No sound, no camera.** The emotions library's sounds are not played; the camera is
  not exposed. A separate node could play the `.wav` of a move when the action starts,
  and a camera node could publish the Arducam stream (`camera_optical` is in the model).
* **Single serial owner.** The driver and the daemon cannot coexist; the systemd unit
  enforces this with `Conflicts=`, but a driver started by hand while the daemon runs
  fails at `rmc_motors_open` / `ReachyMiniMotorController(...)`.
* **Jitter not measured.** The tick is a ROS wall timer on a 3-thread executor (loop,
  action, subscriptions). Nothing records how often the 20 ms period is missed on the
  Raspberry Pi; the daemon's loop statistics have no equivalent. A `~/loop_period`
  diagnostic would be cheap to add.
* **No servo health.** The daemon reads the hardware error registers once a second and
  reboots faulty servos; the driver only logs bus errors. `rmc_motors_read_voltages` is
  exposed but unused.
* **Python driver optional.** It duplicates the C++ driver and needs the SDK installed
  by hand on Ubuntu 26.04; the metapackage does not pull it in. It stays useful for
  experiments that want the SDK's other tools (`create_head_pose`, `PlacoKinematics`).
* **Passive joints not published.** The platform's ball joints are absent from the
  model; only the head and the rods are placed, from FK and geometry. Enough for RViz
  and for `camera` in TF, not for a collision model of the platform.
* **No speed limits in Python.** The limiter of the C++ driver (section 4, *Speed
  limits*) has no Python counterpart: the Python driver still writes its targets as
  they come.
* **Unreachable targets are silent.** They are dropped with a throttled warning and the
  robot holds its last goal; a caller learns nothing but the log. A service that
  validates a pose would help behaviour nodes.
