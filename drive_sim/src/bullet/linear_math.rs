use glam::{Quat, Vec3A, Vec4};

pub trait QuatExt {
    fn from_angle_axis_up(angle: f32) -> Self;
}

impl QuatExt for Quat {
    #[inline]
    fn from_angle_axis_up(angle: f32) -> Self {
        let (s, c) = (angle * 0.5).sin_cos();
        Self::from_xyzw(0.0, 0.0, s, c)
    }
}

pub fn to_simd(nums: &[[f32; 4]; 3]) -> [Vec4; 3] {
    [
        Vec4::from_array(nums[0]),
        Vec4::from_array(nums[1]),
        Vec4::from_array(nums[2]),
    ]
}

pub fn from_simd(vecs: &[Vec4; 3]) -> [Vec3A; 4] {
    [
        Vec3A::new(vecs[0].x, vecs[1].x, vecs[2].x),
        Vec3A::new(vecs[0].y, vecs[1].y, vecs[2].y),
        Vec3A::new(vecs[0].z, vecs[1].z, vecs[2].z),
        Vec3A::new(vecs[0].w, vecs[1].w, vecs[2].w),
    ]
}
