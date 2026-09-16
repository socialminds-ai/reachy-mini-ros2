//! Vendored from the `reachy_mini_rust_kinematics` 1.0.3 source distribution
//! (Pollen Robotics): analytical inverse kinematics and numerical forward
//! kinematics of the Reachy Mini Stewart platform. The pyo3 wrapper was
//! removed; the algorithm is untouched. See VENDORING.md.
use nalgebra::{DVector, Matrix3, Matrix3x6, Matrix4, MatrixXx6, Vector3};

pub const HEAD_Z_OFFSET: f64 = 0.177;

struct Branch {
    branch_platform: Vector3<f64>,
    t_world_motor: Matrix4<f64>,
    solution: f64,
    jacobian: Matrix3x6<f64>,
}

pub struct Kinematics {
    motor_arm_length: f64,
    rod_length: f64,
    t_world_platform: Matrix4<f64>,
    line_search_maximum_iterations: usize,
    branches: Vec<Branch>,
    body_yaw: f64,
}

impl Kinematics {
    pub fn new(motor_arm_length: f64, rod_length: f64) -> Self {
        let t_world_platform = Matrix4::identity();
        let line_search_maximum_iterations = 16;

        let branches = Vec::new();
        Self {
            motor_arm_length,
            rod_length,
            t_world_platform,
            line_search_maximum_iterations,
            branches,
            body_yaw: 0.0,
        }
    }

    pub fn add_branch(
        &mut self,
        branch_platform: Vector3<f64>,
        t_world_motor: Matrix4<f64>,
        solution: f64,
    ) {
        // Building a 3x6 jacobian relating platform velocity to branch anchor point
        // linear velocity Linear velocity is kept as identity and angular velocity is
        // using Varignon's formula w x p, which Is anti-symmetric -p x w and used in
        // matrix form [-p]

        let mut jacobian: Matrix3x6<f64> = Matrix3x6::zeros();
        let mut slice = jacobian.view_mut((0, 0), (3, 3));
        slice += Matrix3::identity();
        let p = -branch_platform;
        let mut slice = jacobian.view_mut((0, 3), (3, 3));
        slice[(0, 1)] = -p.z;
        slice[(0, 2)] = p.y;
        slice[(1, 0)] = p.z;
        slice[(1, 2)] = -p.x;
        slice[(2, 0)] = -p.y;
        slice[(2, 1)] = p.x;

        self.branches.push(Branch {
            branch_platform,
            t_world_motor,
            solution,
            jacobian,
        });
    }

    fn wrap_angle(angle: f64) -> f64 {
        angle
            - (2.0 * std::f64::consts::PI)
                * ((angle + std::f64::consts::PI) * (1.0 / (2.0 * std::f64::consts::PI))).floor()
    }

    pub fn inverse_kinematics_safe(
        &mut self,
        t_world_platform: Matrix4<f64>,
        body_yaw: Option<f64>,
        max_relative_yaw: Option<f64>,
        max_body_yaw: Option<f64>,
    ) -> Vec<f64> {
        let mut joint_angles: Vec<f64> = vec![0.0; self.branches.len() + 1];
        let mut body_yaw_target = 0.0;
        // if body yaw is specified, rotate the platform accordingly
        if body_yaw.is_some() {
            body_yaw_target = -body_yaw.unwrap();
            // first verify if the body yaw is within the allowed limits
            // relative yaw is the yaw difference between the current platform yaw and body yaw
            // it should stays within +/- max_relative_yaw
            if let Some(max_rel_yaw) = max_relative_yaw {
                let current_yaw = t_world_platform[(0, 1)].atan2(t_world_platform[(0, 0)]);
                let relative_yaw = body_yaw_target - current_yaw;
                body_yaw_target = current_yaw + relative_yaw.clamp(-max_rel_yaw, max_rel_yaw);
            }
            // then clamp the body yaw within +/- max_body_yaw
            // this is physically limited by the mechanical design
            if let Some(max_body_yaw) = max_body_yaw {
                body_yaw_target = body_yaw_target.clamp(-max_body_yaw, max_body_yaw);
            }
            body_yaw_target = -body_yaw_target;
        }

        // construct the joint angles vector
        joint_angles[0] = body_yaw_target;
        joint_angles[1..]
            .copy_from_slice(&self.inverse_kinematics(t_world_platform, Some(body_yaw_target)));
        joint_angles
    }

    #[allow(non_snake_case)]
    pub fn inverse_kinematics(
        &mut self,
        t_world_platform: Matrix4<f64>,
        body_yaw: Option<f64>,
    ) -> Vec<f64> {
        let mut joint_angles: Vec<f64> = vec![0.0; self.branches.len()];
        let rs = self.motor_arm_length;
        let rp = self.rod_length;

        let mut t_world_platform_target = t_world_platform;

        // if body yaw is specified, rotate the platform accordingly
        if body_yaw.is_some() {
            let yaw = body_yaw.unwrap();
            let rotation = nalgebra::Rotation3::from_axis_angle(
                &nalgebra::Unit::new_normalize(Vector3::z()),
                -yaw,
            );
            let t_yaw = rotation.to_homogeneous();
            t_world_platform_target = t_yaw * t_world_platform;
        }

        for (k, branch) in self.branches.iter().enumerate() {
            let t_world_motor_inv = branch.t_world_motor.try_inverse().unwrap();
            let branch_motor = t_world_motor_inv
                * t_world_platform_target
                * Matrix4::new(
                    1.0,
                    0.0,
                    0.0,
                    branch.branch_platform.x,
                    0.0,
                    1.0,
                    0.0,
                    branch.branch_platform.y,
                    0.0,
                    0.0,
                    1.0,
                    branch.branch_platform.z,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                );
            let px = branch_motor[(0, 3)];
            let py = branch_motor[(1, 3)];
            let pz = branch_motor[(2, 3)];

            let x = px.powi(2) + 2.0 * px * rs + py.powi(2) + pz.powi(2) - rp.powi(2) + rs.powi(2);
            let y = 2.0 * py * rs
                + branch.solution
                    * (-(px.powi(4))
                        - 2.0 * px.powi(2) * py.powi(2)
                        - 2.0 * px.powi(2) * pz.powi(2)
                        + 2.0 * px.powi(2) * rp.powi(2)
                        + 2.0 * px.powi(2) * rs.powi(2)
                        - py.powi(4)
                        - 2.0 * py.powi(2) * pz.powi(2)
                        + 2.0 * py.powi(2) * rp.powi(2)
                        + 2.0 * py.powi(2) * rs.powi(2)
                        - pz.powi(4)
                        + 2.0 * pz.powi(2) * rp.powi(2)
                        - 2.0 * pz.powi(2) * rs.powi(2)
                        - rp.powi(4)
                        + 2.0 * rp.powi(2) * rs.powi(2)
                        - rs.powi(4))
                    .sqrt();

            joint_angles[k] = Self::wrap_angle(2.0 * y.atan2(x));
        }
        joint_angles
    }

    pub fn reset_forward_kinematics(&mut self, t_world_platform: Matrix4<f64>) {
        self.t_world_platform = t_world_platform;
    }

    #[allow(non_snake_case)]
    pub fn forward_kinematics(
        &mut self,
        joint_angles: Vec<f64>,
        body_yaw: Option<f64>,
    ) -> Matrix4<f64> {
        if self.branches.len() != 6 {
            panic!("Forward kinematics requires exactly 6 joint angles");
        }

        let mut J = MatrixXx6::<f64>::zeros(6);
        let mut errors = DVector::<f64>::zeros(6);
        let mut arms_motor: Vec<Vector3<f64>> = Vec::new();

        for k in 0..self.branches.len() {
            let branch = &self.branches[k];

            // Computing the position of motor arm in the motor frame
            let arm_motor = self.motor_arm_length
                * Vector3::new(joint_angles[k].cos(), joint_angles[k].sin(), 0.0);
            arms_motor.push(arm_motor);

            // Expressing the tip of motor arm in the platform frame
            // Convert arm_motor to homogeneous coordinates for multiplication
            let arm_motor_hom = arm_motor.push(1.0);
            let arm_platform_hom =
                self.t_world_platform.try_inverse().unwrap() * branch.t_world_motor * arm_motor_hom;
            let arm_platform = arm_platform_hom.fixed_rows::<3>(0).into_owned();

            // Computing the current distance
            let current_distance = (arm_platform - branch.branch_platform).norm();

            // Computing the arm-to-branch vector in platform frame
            let arm_branch_platform: Vector3<f64> = branch.branch_platform - arm_platform;

            // Computing the jacobian of the distance
            let mut slice = J.view_mut((k, 0), (1, 6));
            slice += arm_branch_platform.transpose() * branch.jacobian;
            errors[k] = self.rod_length - current_distance;
        }

        // If the error is sufficiently high, performs a line-search along the direction given by the jacobian inverse
        if errors.norm() > 1e-6 {
            let mut V = J.pseudo_inverse(1e-6).unwrap() * errors.clone();
            for _i in 0..self.line_search_maximum_iterations {
                let mut T: Matrix4<f64> = Matrix4::identity();
                T[(0, 3)] = V[0];
                T[(1, 3)] = V[1];
                T[(2, 3)] = V[2];

                let norm = V.fixed_rows::<3>(3).norm();
                if norm.abs() > 1e-6 {
                    let tail = V.fixed_rows::<3>(3).normalize();
                    let axis = nalgebra::Unit::new_normalize(tail);
                    let rotation = nalgebra::Rotation3::from_axis_angle(&axis, norm);
                    let linear = rotation.matrix();
                    let mut slice = T.view_mut((0, 0), (3, 3));
                    slice.copy_from(linear);
                }
                let t_world_platform2 = self.t_world_platform * T;

                let mut new_errors = DVector::<f64>::zeros(self.branches.len());
                for k in 0..self.branches.len() {
                    let branch = &self.branches[k];

                    let arm_motor_hom = arms_motor[k].push(1.0);
                    let arm_platform_hom = t_world_platform2.try_inverse().unwrap()
                        * branch.t_world_motor
                        * arm_motor_hom;
                    let arm_platform = arm_platform_hom.fixed_rows::<3>(0).into_owned();
                    let current_distance = (arm_platform - branch.branch_platform).norm();

                    new_errors[k] = self.rod_length - current_distance;
                }

                if new_errors.norm() < errors.norm() {
                    self.t_world_platform = t_world_platform2;
                    break;
                } else {
                    for j in 0..V.len() {
                        V[j] *= 0.5;
                    }
                }
            }
        }

        // prepare the retun value by applying body yaw if specified
        let mut t_world_platform = self.t_world_platform;

        // rotate the body around Z if body_yaw is specified
        if let Some(yaw) = body_yaw {
            // remove the z offset
            t_world_platform[(2, 3)] -= HEAD_Z_OFFSET;
            // rotate
            let rotation = nalgebra::Rotation3::from_axis_angle(
                &nalgebra::Unit::new_normalize(Vector3::z()),
                yaw,
            );
            let t_yaw = rotation.to_homogeneous();
            t_world_platform = t_yaw * t_world_platform;
            // re-apply the z offset
            t_world_platform[(2, 3)] += HEAD_Z_OFFSET;
        }

        t_world_platform
    }
}
