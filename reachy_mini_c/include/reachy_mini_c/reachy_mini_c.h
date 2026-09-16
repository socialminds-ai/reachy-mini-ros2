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

/*
 * reachy_mini_c - C API over Pollen Robotics' Reachy Mini motor controller and
 * head kinematics (implemented in Rust, see rust/).
 *
 * Conventions (identical to the Python SDK):
 *  - 9-element motor arrays: [body_yaw, stewart_1..stewart_6, right_antenna, left_antenna], radians
 *  - 7-element joint arrays: [body_yaw, stewart_1..stewart_6], radians
 *  - poses: 4x4 homogeneous matrices, row-major, 16 doubles, head frame in the
 *    robot base frame (same as ReachyMini.set_target(head=...) and the
 *    recorded moves of the emotions library).
 *
 * Every function returning int returns 0 on success and -1 on error; the error
 * message is then available from rmc_last_error() (thread-local).
 */
#ifndef REACHY_MINI_C__REACHY_MINI_C_H_
#define REACHY_MINI_C__REACHY_MINI_C_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RMC_NUM_MOTORS 9
#define RMC_NUM_JOINTS 7

/** Message of the last error on this thread ("" if none). */
const char * rmc_last_error(void);
/** Library version string. */
const char * rmc_version(void);

/* ------------------------------------------------------------------------ */
/* Motors                                                                   */
/* ------------------------------------------------------------------------ */
typedef struct RmcMotors RmcMotors;

/** Open the servo bus on a serial port (e.g. "/dev/ttyAMA3"). NULL on error. */
RmcMotors * rmc_motors_open(const char *serial_port);
/** Close the bus and free the handle (torque is left unchanged). */
void rmc_motors_close(RmcMotors *motors);
/** Read the 9 present positions (radians). */
int rmc_motors_read_positions(RmcMotors *motors, double out[RMC_NUM_MOTORS]);
/** Write the 9 goal positions (radians). */
int rmc_motors_write_goal_positions(RmcMotors *motors, const double goals[RMC_NUM_MOTORS]);
/** Enable (enable != 0) or disable torque on all motors. */
int rmc_motors_set_torque(RmcMotors *motors, int enable);
/** Writes 1 into *out if torque is enabled on all motors, 0 otherwise. */
int rmc_motors_get_torque(RmcMotors *motors, int *out);
/** Read the 9 input voltages (0.1 V units). */
int rmc_motors_read_voltages(RmcMotors *motors, uint16_t out[RMC_NUM_MOTORS]);

/* ------------------------------------------------------------------------ */
/* Kinematics                                                               */
/* ------------------------------------------------------------------------ */
typedef struct RmcKinematics RmcKinematics;

/** Solver for the standard Reachy Mini geometry (embedded). NULL on error. */
RmcKinematics * rmc_kinematics_new(void);
void rmc_kinematics_free(RmcKinematics *k);
/**
 * Inverse kinematics with automatic body yaw (SDK default): the body rotation
 * is adjusted so the head/body relative yaw stays within mechanical limits.
 */
int rmc_kinematics_ik(
  RmcKinematics *k, const double head_pose[16], double body_yaw,
  double out_joints[RMC_NUM_JOINTS]);
/** Inverse kinematics with the body yaw taken as is. */
int rmc_kinematics_ik_fixed_yaw(
  RmcKinematics *k, const double head_pose[16], double body_yaw,
  double out_joints[RMC_NUM_JOINTS]);
/**
 * Forward kinematics (iterative, warm-started from the previous result).
 * iterations >= 1; the SDK uses 3 in its control loop and 20 for a cold start.
 */
int rmc_kinematics_fk(
  RmcKinematics *k, const double joints[RMC_NUM_JOINTS], int iterations,
  double out_pose[16]);
/** Reset the FK warm start to the given head pose. */
int rmc_kinematics_reset_fk(RmcKinematics *k, const double head_pose[16]);
/** Platform geometry: motor arm length, rod length, head frame height at rest (m). */
int rmc_kinematics_lengths(
  RmcKinematics *k, double *out_arm_length, double *out_rod_length, double *out_head_z_offset);
/**
 * Geometry of leg `index` (0..5, stewart joint order): motor frame in the base
 * frame (row-major 4x4) and the rod anchor on the platform, in the head frame.
 * The horn tip is at arm_length * (cos(theta), sin(theta), 0) in the motor frame.
 */
int rmc_kinematics_leg(
  RmcKinematics *k, int index, double out_t_base_motor[16], double out_anchor_head[3]);

#ifdef __cplusplus
}
#endif
#endif  // REACHY_MINI_C__REACHY_MINI_C_H_
