use glam::{Vec3A, Vec3Swizzles};

const CONVEX_DISTANCE_MARGIN: f32 = 0.04;

pub struct BoxShape {
    implicit_dim: Vec3A,
}

impl BoxShape {
    pub fn new(box_half_extents: Vec3A) -> Self {
        Self {
            implicit_dim: box_half_extents - CONVEX_DISTANCE_MARGIN,
        }
    }

    #[inline]
    pub fn get_half_extents(&self) -> Vec3A {
        self.implicit_dim
    }

    pub fn calculate_local_intertia(&self, mass: f32) -> Vec3A {
        let l = 2.0 * self.get_half_extents();
        let yxx = l.yxx();
        let zzy = l.zzy();

        mass / 12.0 * (yxx * yxx + zzy * zzy)
    }
}
