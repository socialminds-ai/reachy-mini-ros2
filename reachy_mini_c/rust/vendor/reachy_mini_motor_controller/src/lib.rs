//! Vendored from the `reachy_mini_motor_controller` 1.5.6 source distribution
//! (Pollen Robotics). Only the synchronous controller is kept; the Python
//! bindings and the asynchronous control loop were removed. See VENDORING.md.
mod controller;
pub use controller::ReachyMiniMotorController;
