use glam::{Vec3A, Vec3Swizzles};

const CONVEX_DISTANCE_MARGIN: f32 = 0.04;

pub struct BoxShape {
    implicit_dim: Vec3A,
    margin: f32,
}

impl BoxShape {
    #[inline]
    pub fn new(box_half_extents: Vec3A) -> Self {
        let safe_margin = 0.1 * box_half_extents.min_element();
        let margin = safe_margin.min(CONVEX_DISTANCE_MARGIN);
        Self {
            implicit_dim: box_half_extents - margin,
            margin,
        }
    }

    #[inline]
    const fn get_half_extents(&self) -> Vec3A {
        self.implicit_dim
    }

    pub const fn get_margin(&self) -> f32 {
        self.margin
    }

    pub fn calculate_local_intertia(&self, mass: f32) -> Vec3A {
        let l = 2.0 * (self.get_half_extents() + self.get_margin());
        let yxx = l.yxx();
        let zzy = l.zzy();

        mass / 12.0 * (yxx * yxx + zzy * zzy)
    }
}
