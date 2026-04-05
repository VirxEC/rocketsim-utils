use glam::Vec3A;

use crate::{bullet::linear_math::LARGE_FLOAT, shared::Aabb};

pub struct StaticPlaneShape {
    plane_normal: Vec3A,
    pub aabb_cache: Aabb,
}

impl StaticPlaneShape {
    pub fn new(world_trans: Vec3A, plane_normal: Vec3A) -> Self {
        debug_assert!(plane_normal.is_normalized());

        let [x, y, z]: [bool; 3] = plane_normal.abs().cmpge(Vec3A::splat(f32::EPSILON)).into();

        let (is_single_axis, single_axis_idx, single_axis_backwards) =
            if u8::from(x) + u8::from(y) + u8::from(z) == 1 {
                let axis = plane_normal.abs().max_position();

                (true, axis, plane_normal[axis].is_sign_negative())
            } else {
                (false, 0, false)
            };
        debug_assert!(is_single_axis);

        let aabb_cache = Self::get_aabb(single_axis_idx, single_axis_backwards, world_trans);

        Self {
            plane_normal,
            aabb_cache,
        }
    }

    fn get_aabb(single_axis_idx: usize, single_axis_backwards: bool, t: Vec3A) -> Aabb {
        const PLANE_CONSTANT_OFFSET: f32 = 0.2;

        let mut min = Vec3A::splat(-LARGE_FLOAT);
        let mut max = Vec3A::splat(LARGE_FLOAT);

        min[single_axis_idx] = t[single_axis_idx] - PLANE_CONSTANT_OFFSET;
        max[single_axis_idx] = t[single_axis_idx] + PLANE_CONSTANT_OFFSET;

        (if single_axis_backwards {
            &mut max
        } else {
            &mut min
        })[single_axis_idx] = if single_axis_backwards {
            LARGE_FLOAT
        } else {
            -LARGE_FLOAT
        };

        Aabb { min, max }
    }

    pub const fn get_plane_normal(&self) -> Vec3A {
        self.plane_normal
    }
}
