use std::f32::consts::FRAC_1_SQRT_2;

use glam::{Quat, Vec3A};
pub mod angle;

pub const LARGE_FLOAT: f32 = 1e18;

pub trait QuatExt {
    fn from_axis_angle_simd(axis: Vec3A, angle: f32) -> Self;
}

impl QuatExt for Quat {
    #[inline]
    /// An implementation of `Quat::from_axis_angle` that leverages simd
    fn from_axis_angle_simd(axis: Vec3A, angle: f32) -> Self {
        debug_assert!(axis.is_normalized());
        let (s, c) = f32::sin_cos(angle * 0.5);
        let v = axis * s;
        Self::from_xyzw(v.x, v.y, v.z, c)
    }
}

pub fn plane_space_1(n: Vec3A) -> Vec3A {
    if n.z.abs() > FRAC_1_SQRT_2 {
        // choose p in y-z plane
        let a = n.y * n.y + n.z * n.z;
        let k = a.sqrt().recip();
        Vec3A::new(0., -n.z * k, n.y * k)
    } else {
        // choose p in x-y plane
        let a = n.x * n.x + n.y * n.y;
        let k = a.sqrt().recip();
        Vec3A::new(-n.y * k, n.x * k, 0.)
    }
}
