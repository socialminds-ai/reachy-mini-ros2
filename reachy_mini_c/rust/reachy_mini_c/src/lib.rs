//! `reachy_mini_c`: a small, stable C ABI over Pollen Robotics' Rust crates for
//! the Reachy Mini, so that C and C++ programs (for instance ROS 2 nodes) can
//! drive the servos and compute the head kinematics without Python.
//!
//! Conventions (identical to the Python SDK):
//! * Motor order for all 9-element arrays:
//!   `[body_yaw, stewart_1..stewart_6, right_antenna, left_antenna]`, radians.
//! * Kinematic joints for all 7-element arrays: `[body_yaw, stewart_1..6]`, radians.
//! * Poses are 4x4 homogeneous matrices, **row-major**, 16 doubles, expressing
//!   the head frame in the robot base frame, exactly like `ReachyMini.set_target(head=...)`
//!   and the recorded moves of the emotions library. The 0.177 m vertical
//!   offset between the platform and the head frame is handled internally.
//!
//! Every function returning `int` returns 0 on success and -1 on error; the
//! error message is then available from `rmc_last_error()` (thread-local).

use std::cell::RefCell;
use std::ffi::{CStr, CString, c_char};
use std::os::raw::c_int;

use nalgebra::{Matrix4, Vector3};
use reachy_mini_kinematics::{HEAD_Z_OFFSET, Kinematics};
use reachy_mini_motor_controller::ReachyMiniMotorController;
use serde::Deserialize;

const VERSION: &str = concat!(env!("CARGO_PKG_VERSION"), "\0");
const KINEMATICS_DATA: &str = include_str!("../kinematics_data.json");

thread_local! {
    static LAST_ERROR: RefCell<CString> = RefCell::new(CString::new("").unwrap());
}

fn set_error(msg: impl ToString) -> c_int {
    let s = CString::new(msg.to_string()).unwrap_or_else(|_| CString::new("error").unwrap());
    LAST_ERROR.with(|e| *e.borrow_mut() = s);
    -1
}

/// Message of the last error that happened on this thread ("" if none).
/// The pointer stays valid until the next failing call on the same thread.
#[unsafe(no_mangle)]
pub extern "C" fn rmc_last_error() -> *const c_char {
    LAST_ERROR.with(|e| e.borrow().as_ptr())
}

/// Library version, as a static NUL-terminated string.
#[unsafe(no_mangle)]
pub extern "C" fn rmc_version() -> *const c_char {
    VERSION.as_ptr() as *const c_char
}

// ---------------------------------------------------------------------------
// Motors
// ---------------------------------------------------------------------------

/// Opaque handle on the servo bus.
pub struct RmcMotors {
    inner: ReachyMiniMotorController,
}

/// Open the servo bus on the given serial port (e.g. "/dev/ttyAMA3").
/// Returns NULL on failure (see `rmc_last_error`).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rmc_motors_open(serial_port: *const c_char) -> *mut RmcMotors {
    if serial_port.is_null() {
        set_error("serial_port is NULL");
        return std::ptr::null_mut();
    }
    let port = match unsafe { CStr::from_ptr(serial_port) }.to_str() {
        Ok(p) => p,
        Err(e) => {
            set_error(e);
            return std::ptr::null_mut();
        }
    };
    match ReachyMiniMotorController::new(port) {
        Ok(inner) => Box::into_raw(Box::new(RmcMotors { inner })),
        Err(e) => {
            set_error(format!("cannot open {port}: {e}"));
            std::ptr::null_mut()
        }
    }
}

/// Close the bus and free the handle. Torque is left as it is.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rmc_motors_close(motors: *mut RmcMotors) {
    if !motors.is_null() {
        drop(unsafe { Box::from_raw(motors) });
    }
}

macro_rules! check_handle {
    ($h:expr) => {
        match unsafe { $h.as_mut() } {
            Some(m) => m,
            None => return set_error("motors handle is NULL"),
        }
    };
}

/// Read the present position of the 9 motors (radians).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rmc_motors_read_positions(motors: *mut RmcMotors, out: *mut f64) -> c_int {
    let m = check_handle!(motors);
    if out.is_null() {
        return set_error("out is NULL");
    }
    match m.inner.read_all_positions() {
        Ok(pos) => {
            unsafe { std::ptr::copy_nonoverlapping(pos.as_ptr(), out, 9) };
            0
        }
        Err(e) => set_error(format!("read_positions: {e}")),
    }
}

/// Write the goal position of the 9 motors (radians).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rmc_motors_write_goal_positions(motors: *mut RmcMotors, goals: *const f64) -> c_int {
    let m = check_handle!(motors);
    if goals.is_null() {
        return set_error("goals is NULL");
    }
    let mut arr = [0.0f64; 9];
    unsafe { std::ptr::copy_nonoverlapping(goals, arr.as_mut_ptr(), 9) };
    match m.inner.set_all_goal_positions(arr) {
        Ok(()) => 0,
        Err(e) => set_error(format!("write_goal_positions: {e}")),
    }
}

/// Enable (`enable != 0`) or disable torque on all motors.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rmc_motors_set_torque(motors: *mut RmcMotors, enable: c_int) -> c_int {
    let m = check_handle!(motors);
    let r = if enable != 0 { m.inner.enable_torque() } else { m.inner.disable_torque() };
    match r {
        Ok(()) => 0,
        Err(e) => set_error(format!("set_torque: {e}")),
    }
}

/// Is torque enabled on all motors? Writes 1 or 0 into `out`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rmc_motors_get_torque(motors: *mut RmcMotors, out: *mut c_int) -> c_int {
    let m = check_handle!(motors);
    if out.is_null() {
        return set_error("out is NULL");
    }
    match m.inner.is_torque_enabled() {
        Ok(on) => {
            unsafe { *out = on as c_int };
            0
        }
        Err(e) => set_error(format!("get_torque: {e}")),
    }
}

/// Read the input voltage of the 9 motors (in 0.1 V units, as reported by the servos).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rmc_motors_read_voltages(motors: *mut RmcMotors, out: *mut u16) -> c_int {
    let m = check_handle!(motors);
    if out.is_null() {
        return set_error("out is NULL");
    }
    match m.inner.read_all_voltages() {
        Ok(v) => {
            unsafe { std::ptr::copy_nonoverlapping(v.as_ptr(), out, 9) };
            0
        }
        Err(e) => set_error(format!("read_voltages: {e}")),
    }
}

// ---------------------------------------------------------------------------
// Kinematics
// ---------------------------------------------------------------------------

#[derive(Deserialize)]
struct MotorGeometry {
    branch_position: [f64; 3],
    #[serde(rename = "T_motor_world")]
    t_motor_world: [[f64; 4]; 4],
    solution: serde_json::Value, // 0/1 or true/false depending on the SDK version
}

#[derive(Deserialize)]
struct KinematicsData {
    motor_arm_length: f64,
    rod_length: f64,
    head_z_offset: f64,
    motors: Vec<MotorGeometry>,
}

/// Opaque handle on a kinematics solver.
pub struct RmcKinematics {
    kin: Kinematics,
    head_z_offset: f64,
}

fn mat_from_row_major(p: *const f64) -> Matrix4<f64> {
    let mut m = Matrix4::<f64>::zeros();
    for r in 0..4 {
        for c in 0..4 {
            m[(r, c)] = unsafe { *p.add(r * 4 + c) };
        }
    }
    m
}

fn mat_to_row_major(m: &Matrix4<f64>, out: *mut f64) {
    for r in 0..4 {
        for c in 0..4 {
            unsafe { *out.add(r * 4 + c) = m[(r, c)] };
        }
    }
}

/// Create a solver for the standard Reachy Mini geometry (embedded in the
/// library, same data as the Python SDK). Returns NULL on failure.
#[unsafe(no_mangle)]
pub extern "C" fn rmc_kinematics_new() -> *mut RmcKinematics {
    let data: KinematicsData = match serde_json::from_str(KINEMATICS_DATA) {
        Ok(d) => d,
        Err(e) => {
            set_error(format!("embedded kinematics data: {e}"));
            return std::ptr::null_mut();
        }
    };
    let mut kin = Kinematics::new(data.motor_arm_length, data.rod_length);
    for m in &data.motors {
        let t_motor_world = Matrix4::from_fn(|r, c| m.t_motor_world[r][c]);
        let t_world_motor = match t_motor_world.try_inverse() {
            Some(t) => t,
            None => {
                set_error("embedded kinematics data: singular T_motor_world");
                return std::ptr::null_mut();
            }
        };
        let branch = Vector3::new(m.branch_position[0], m.branch_position[1], m.branch_position[2]);
        let solution = match &m.solution {
            serde_json::Value::Bool(b) => *b,
            serde_json::Value::Number(n) => n.as_f64().unwrap_or(0.0) != 0.0,
            _ => {
                set_error("embedded kinematics data: bad 'solution' field");
                return std::ptr::null_mut();
            }
        };
        kin.add_branch(branch, t_world_motor, if solution { 1.0 } else { -1.0 });
    }
    // Same initial guess as the SDK: the sleep pose, shifted by the head offset.
    let mut init = Matrix4::<f64>::identity();
    init[(2, 3)] = data.head_z_offset;
    kin.reset_forward_kinematics(init);
    let _ = HEAD_Z_OFFSET; // documented constant, the JSON value is authoritative
    Box::into_raw(Box::new(RmcKinematics { kin, head_z_offset: data.head_z_offset }))
}

/// Free a solver.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rmc_kinematics_free(k: *mut RmcKinematics) {
    if !k.is_null() {
        drop(unsafe { Box::from_raw(k) });
    }
}

macro_rules! check_kin {
    ($h:expr) => {
        match unsafe { $h.as_mut() } {
            Some(m) => m,
            None => return set_error("kinematics handle is NULL"),
        }
    };
}

/// Inverse kinematics with automatic body yaw (the SDK default): the body
/// rotation is adjusted so that the head/body relative yaw stays within
/// mechanical limits. `head_pose` is a row-major 4x4 matrix, `body_yaw` the
/// requested body yaw (radians). Writes 7 joints into `out_joints`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rmc_kinematics_ik(
    k: *mut RmcKinematics,
    head_pose: *const f64,
    body_yaw: f64,
    out_joints: *mut f64,
) -> c_int {
    let s = check_kin!(k);
    if head_pose.is_null() || out_joints.is_null() {
        return set_error("head_pose or out_joints is NULL");
    }
    let mut pose = mat_from_row_major(head_pose);
    pose[(2, 3)] += s.head_z_offset;
    let joints = s.kin.inverse_kinematics_safe(
        pose,
        Some(body_yaw),
        Some(65.0_f64.to_radians()),
        Some(160.0_f64.to_radians()),
    );
    if joints.len() != 7 || joints.iter().any(|v| !v.is_finite()) {
        return set_error("ik: pose not reachable");
    }
    unsafe { std::ptr::copy_nonoverlapping(joints.as_ptr(), out_joints, 7) };
    0
}

/// Inverse kinematics with a fixed body yaw (no automatic adjustment).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rmc_kinematics_ik_fixed_yaw(
    k: *mut RmcKinematics,
    head_pose: *const f64,
    body_yaw: f64,
    out_joints: *mut f64,
) -> c_int {
    let s = check_kin!(k);
    if head_pose.is_null() || out_joints.is_null() {
        return set_error("head_pose or out_joints is NULL");
    }
    let mut pose = mat_from_row_major(head_pose);
    pose[(2, 3)] += s.head_z_offset;
    let stewart = s.kin.inverse_kinematics(pose, Some(body_yaw));
    if stewart.len() != 6 || stewart.iter().any(|v| !v.is_finite()) {
        return set_error("ik_fixed_yaw: pose not reachable");
    }
    unsafe {
        *out_joints = body_yaw;
        std::ptr::copy_nonoverlapping(stewart.as_ptr(), out_joints.add(1), 6);
    }
    0
}

/// Forward kinematics: 7 joints -> row-major 4x4 head pose. The solver is
/// iterative and warm-started from its previous result; `iterations` (>= 1,
/// 3 is what the SDK uses) refines the solution.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rmc_kinematics_fk(
    k: *mut RmcKinematics,
    joints: *const f64,
    iterations: c_int,
    out_pose: *mut f64,
) -> c_int {
    let s = check_kin!(k);
    if joints.is_null() || out_pose.is_null() {
        return set_error("joints or out_pose is NULL");
    }
    if iterations < 1 {
        return set_error("iterations must be >= 1");
    }
    let j: Vec<f64> = (0..7).map(|i| unsafe { *joints.add(i) }).collect();
    let mut pose = Matrix4::<f64>::identity();
    for _ in 0..iterations {
        pose = s.kin.forward_kinematics(j[1..].to_vec(), Some(j[0]));
    }
    if pose.iter().any(|v| !v.is_finite()) {
        return set_error("fk: did not converge");
    }
    pose[(2, 3)] -= s.head_z_offset;
    mat_to_row_major(&pose, out_pose);
    0
}

/// Reset the forward-kinematics warm start to the given head pose (row-major 4x4).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rmc_kinematics_reset_fk(k: *mut RmcKinematics, head_pose: *const f64) -> c_int {
    let s = check_kin!(k);
    if head_pose.is_null() {
        return set_error("head_pose is NULL");
    }
    let mut pose = mat_from_row_major(head_pose);
    pose[(2, 3)] += s.head_z_offset;
    s.kin.reset_forward_kinematics(pose);
    0
}
