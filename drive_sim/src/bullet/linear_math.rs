use glam::{Quat, Vec4};

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
