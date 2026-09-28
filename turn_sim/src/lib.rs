use std::f32::consts::FRAC_PI_4;

use glam::{IVec3, Mat3A, Quat, Vec3A};

use crate::{car_controls::CarControls, consts::car};

pub mod car_controls;
pub mod consts;

/// Asymmetric 8-bit input quantization: negatives scale by 128, positives by 127.
fn quantize_air_inputs(ctrls: Vec3A) -> Vec3A {
    const UPPER_BOUND: Vec3A = Vec3A::splat(128.0);
    const LOWER_BOUND: Vec3A = Vec3A::splat(127.0);

    let clamped = ctrls.clamp(Vec3A::NEG_ONE, Vec3A::ONE);
    let scale = Vec3A::select(clamped.cmplt(Vec3A::ZERO), UPPER_BOUND, LOWER_BOUND);
    let biased = clamped * scale + UPPER_BOUND;
    let w = biased + biased + Vec3A::splat(0.5);
    let byte = ((w.round().as_ivec3() >> 1i32) & IVec3::splat(0xFF)).as_vec3a();
    let s = byte - UPPER_BOUND;

    Vec3A::select(
        s.cmplt(Vec3A::ZERO),
        s * (1.0 / UPPER_BOUND),
        s / LOWER_BOUND,
    )
}

#[derive(Clone, Copy, Debug)]
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
        let ctrls = quantize_air_inputs(Vec3A::new(ctrls.pitch, ctrls.yaw, ctrls.roll));

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
        let (sin_half, cos_half) = half_angle_dt.sin_cos();
        let axis = ang_vel
            * if angle < 0.001 {
                (1.0 - dt * dt * 2.0 * 0.020_833_334) * angle * half_angle_dt
            } else {
                sin_half / angle
            };

        let dorn = Quat::from_xyzw(axis.x, axis.y, axis.z, cos_half);
        (dorn * rot).normalize()
    }
}

#[cfg(test)]
mod tests {
    use rand::{RngExt, SeedableRng, rngs::SmallRng};

    use super::*;

    /// RocketSim's scalar air-control quantization, per axis.
    fn quantize_air_input(x: f32) -> f32 {
        let clamped = x.clamp(-1.0, 1.0);
        let y = if clamped < 0.0 {
            (clamped * 128.0).max(-128.0)
        } else {
            (clamped * 127.0).min(127.0)
        };
        let w = ((y + 128.0) + (y + 128.0)) + 0.5;
        let eax = w.round_ties_even() as i32;
        let byte = ((eax >> 1) & 0xFF) as u8;
        let s = (byte as f32) - 128.0;
        if byte < 0x80 {
            s * (1.0 / 128.0)
        } else {
            s / 127.0
        }
    }

    #[track_caller]
    fn assert_matches_scalar(ctrls: Vec3A) {
        let want = quantize_air_inputs(ctrls).to_array();
        for (axis, (&got, &input)) in want.iter().zip(ctrls.to_array().iter()).enumerate() {
            let expected = quantize_air_input(input);
            assert_eq!(
                got.to_bits(),
                expected.to_bits(),
                "axis {axis}: input {input} gave {got}, RocketSim gives {expected}"
            );
        }
    }

    #[test]
    fn quantize_matches_rocketsim_on_the_full_input_range() {
        // Every representable output boundary, plus the out-of-range inputs the
        // clamp has to absorb.
        for step in 1..=1024 {
            for input in [-1.5, -1.0, 1.0, 1.5] {
                assert_matches_scalar(Vec3A::splat(input * step as f32 / 1024.0));
            }
        }
        assert_matches_scalar(Vec3A::ZERO);
        assert_matches_scalar(Vec3A::new(-1.0, 0.0, 1.0));
        assert_matches_scalar(Vec3A::splat(f32::MIN));
        assert_matches_scalar(Vec3A::splat(f32::MAX));
        assert_matches_scalar(Vec3A::splat(-0.0));
    }

    #[test]
    fn quantize_matches_rocketsim_on_unaligned_inputs() {
        let mut rng = SmallRng::seed_from_u64(0x51A7);
        for i in 0..100_000 {
            // Uniform over the whole f32 range hits the clamp on almost every
            // draw, so weight most of them into [-1.05, 1.05] where all 256
            // output levels and every tie live. NaN and the infinities are the
            // one case `to_bits` can't compare, and the clamp folds them into a
            // unit input anyway, so keep them out of the comparison.
            let inputs = [(); 3].map(|_| {
                if i % 16 == 0 {
                    raw_or_clamped(&mut rng)
                } else {
                    rng.random_range(-1.05..=1.05)
                }
            });
            assert_matches_scalar(Vec3A::from_array(inputs));
        }
    }

    /// A raw `f32` draw, with the non-finite cases replaced by their clamped
    /// equivalents so the results stay comparable bit for bit.
    fn raw_or_clamped(rng: &mut SmallRng) -> f32 {
        let x: f32 = rng.random();
        if x.is_finite() { x } else { x.clamp(-1.0, 1.0) }
    }
}
