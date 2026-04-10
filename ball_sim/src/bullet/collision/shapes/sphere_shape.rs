use glam::Vec3A;

use crate::{
    bullet::collision::narrowphase::persistent_manifold::CONTACT_BREAKING_THRESHOLD, shared::Aabb,
};

pub const SPHERE_RADIUS_MARGIN: f32 = 0.08;

#[derive(Clone, Copy, Debug)]
pub struct SphereShape {
    radius: f32,
    extent: Vec3A,
}

impl SphereShape {
    #[inline]
    pub const fn new(radius: f32) -> Self {
        Self {
            radius,
            extent: Vec3A::splat(radius + SPHERE_RADIUS_MARGIN),
        }
    }

    #[inline]
    pub const fn get_radius(&self) -> f32 {
        self.radius
    }

    #[inline]
    pub const fn get_margin(&self) -> f32 {
        self.get_radius()
    }

    pub fn get_aabb(&self, center: Vec3A) -> Aabb {
        Aabb {
            min: center - self.extent,
            max: center + self.extent,
        }
    }

    pub fn calculate_local_inertia(&self, mass: f32) -> Vec3A {
        Vec3A::splat(0.4 * mass * self.get_margin() * self.get_margin())
    }

    pub fn local_get_supporting_vertex(&self, vec: Vec3A) -> Vec3A {
        self.get_margin() * vec.normalize()
    }

    #[inline]
    pub fn get_contact_breaking_threshold(&self) -> f32 {
        self.extent.x * CONTACT_BREAKING_THRESHOLD
    }
}
