use std::f32::consts::FRAC_PI_4;

use glam::{Quat, Vec3A};

const ANGULAR_MOTION_THRESHOLD: f32 = FRAC_PI_4;

pub fn integrate_trans(rotation: &mut Quat, ang_vel: Vec3A, time_step: f32) {
    let mut angle = ang_vel.length();

    if angle * time_step > ANGULAR_MOTION_THRESHOLD {
        angle = ANGULAR_MOTION_THRESHOLD / time_step;
    }

    let axis = if angle < 0.001 {
        ang_vel
            * (0.5 * time_step - time_step * time_step * time_step * 0.020_833_334)
            * angle
            * angle
    } else {
        ang_vel * ((0.5 * angle * time_step).sin() / angle)
    };

    let dorn = Quat::from_xyzw(axis.x, axis.y, axis.z, (angle * time_step * 0.5).cos());
    *rotation = (dorn * *rotation).normalize();
}
