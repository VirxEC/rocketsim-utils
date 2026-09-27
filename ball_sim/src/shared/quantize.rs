use glam::Vec3A;

use crate::{
    bullet::dynamics::sphere_rigid_body::SphereRigidBody,
    sim::consts::{BT_TO_UU, UU_TO_BT, quantize},
};

/// An extension of `Vec3A::signum` that keeps zero-components as zero (regardless of sign bit).
#[must_use]
fn vec3a_sign_3(v: Vec3A) -> Vec3A {
    let signum = v.signum();
    Vec3A::select(v.cmpeq(Vec3A::ZERO), Vec3A::ZERO, signum)
}

enum VecQuantizeMode {
    Position,
    Velocity,
}

/// UE3-networking-style quantization of vectors.
#[must_use]
fn quantize_vec_ue3(vec: Vec3A, scale: f32, quantize_mode: VecQuantizeMode) -> Vec3A {
    match quantize_mode {
        VecQuantizeMode::Position => (vec * scale + 0.5).floor() / scale,
        VecQuantizeMode::Velocity => {
            let inv_scale = 1.0 / scale;
            let scaled = vec * scale;
            let truncated = scaled.trunc();
            let rounded = truncated * inv_scale + Vec3A::ZERO;

            const OFFSET_CORRECT_FRAC: f32 = 0.1;
            let offset_mag = OFFSET_CORRECT_FRAC * inv_scale;

            rounded + (vec3a_sign_3(truncated) * offset_mag)
        }
    }
}

/// Quantizes the position, linear velocity, and angular velocity of a rigid body.
pub fn quantize(body: &mut SphereRigidBody) {
    let new_pos = quantize_vec_ue3(
        body.get_world_trans() * BT_TO_UU,
        quantize::POS_SCALE,
        VecQuantizeMode::Position,
    ) * UU_TO_BT;
    let new_vel = quantize_vec_ue3(
        body.lin_vel * BT_TO_UU,
        quantize::VEL_SCALE,
        VecQuantizeMode::Velocity,
    ) * UU_TO_BT;
    let new_ang_vel = quantize_vec_ue3(
        body.ang_vel * BT_TO_UU,
        quantize::ANG_VEL_SCALE,
        VecQuantizeMode::Velocity,
    ) * UU_TO_BT;

    body.set_world_trans(new_pos);
    body.set_lin_vel(new_vel);
    body.set_ang_vel(new_ang_vel);
}
