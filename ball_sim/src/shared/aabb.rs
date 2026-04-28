use std::ops::{Add, AddAssign};

use glam::Vec3A;

#[derive(Clone, Copy, Debug, Default)]
pub struct Aabb {
    pub min: Vec3A,
    pub max: Vec3A,
}

impl Aabb {
    pub const ZERO: Self = Self {
        min: Vec3A::ZERO,
        max: Vec3A::ZERO,
    };

    #[inline]
    #[must_use]
    pub const fn new(min: Vec3A, max: Vec3A) -> Self {
        Self { min, max }
    }

    #[inline]
    #[must_use]
    pub fn center(&self) -> Vec3A {
        (self.min + self.max) * 0.5
    }

    #[must_use]
    pub fn area(&self) -> f32 {
        let extents = self.max - self.min;
        2.0 * (extents.x * extents.y + extents.x * extents.z + extents.y * extents.z)
    }

    #[inline]
    #[must_use]
    pub fn intersects(&self, rhs: &Self) -> bool {
        self.min.cmple(rhs.max).all() && self.max.cmpge(rhs.min).all()
    }

    /// Finds the point inside the AABB nearest to the given point
    #[inline]
    #[must_use]
    pub fn clamp(&self, point: Vec3A) -> Vec3A {
        point.clamp(self.min, self.max)
    }
}

impl Add for Aabb {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self {
            min: self.min.min(rhs.min),
            max: self.max.max(rhs.max),
        }
    }
}

impl AddAssign for Aabb {
    fn add_assign(&mut self, rhs: Self) {
        self.min = self.min.min(rhs.min);
        self.max = self.max.max(rhs.max);
    }
}
