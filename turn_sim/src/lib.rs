use std::f32::consts::FRAC_PI_4;

use glam::{Mat3A, Quat, Vec3A};

use crate::{car_controls::CarControls, consts::car};

pub mod car_controls;
pub mod consts;

pub struct Car {
    pub rot: Quat,
    pub ang_vel: Vec3A,
}

impl Car {
    #[inline]
    #[must_use]
    pub fn rot_mat(&self) -> Mat3A {
        Mat3A::from_quat(self.rot)
    }

    pub fn step_turn(&mut self, ctrls: CarControls, dt: f32) {
        const REMAP_BASIS: Quat = Quat::from_array([0.5, -0.5, -0.5, 0.5]);

        let direction = self.rot * REMAP_BASIS;
        let ctrls = Vec3A::new(ctrls.pitch, ctrls.yaw, ctrls.roll);

        let pyr_torque_factor = ctrls * car::air_control::TORQUE;
        let torque = direction * pyr_torque_factor;

        let ctrl_damp_factor = 1.0 - ctrls.with_z(0.0).abs();
        let damp_pyr =
            direction.inverse() * self.ang_vel * car::air_control::DAMPING * ctrl_damp_factor;

        let damping = direction * damp_pyr;
        let total_torque = (torque - damping) * car::air_control::TORQUE_APPLY_SCALE;
        self.ang_vel += total_torque * dt;
        self.rot = Self::integrate_transform(self.rot, self.ang_vel, dt);

        let ang_vel_len_sq = self.ang_vel.length_squared();
        if ang_vel_len_sq > car::MAX_ANG_SPEED * car::MAX_ANG_SPEED {
            self.ang_vel = self.ang_vel / ang_vel_len_sq.sqrt() * car::MAX_ANG_SPEED;
        }
    }

    #[must_use]
    pub fn integrate_transform(rot: Quat, ang_vel: Vec3A, dt: f32) -> Quat {
        const ANGULAR_MOTION_THRESHOLD: f32 = FRAC_PI_4;

        let angle = ang_vel.length().min(ANGULAR_MOTION_THRESHOLD / dt);

        let half_angle_dt = 0.5 * angle * dt;
        let axis = ang_vel
            * if angle < 0.001 {
                (1.0 - dt * dt * 2.0 * 0.020_833_334) * angle * half_angle_dt
            } else {
                half_angle_dt.sin() / angle
            };

        let dorn = Quat::from_xyzw(axis.x, axis.y, axis.z, half_angle_dt.cos());
        (dorn * rot).normalize()
    }
}
