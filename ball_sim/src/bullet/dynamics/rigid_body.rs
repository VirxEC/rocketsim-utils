use glam::Vec3A;

use crate::bullet::collision::{
    narrowphase::persistent_manifold::CONTACT_BREAKING_THRESHOLD,
    shapes::collision_shape::CollisionShapes,
};

pub struct RigidBodyConstructionInfo {
    pub start_world_trans: Vec3A,
    pub collision_shape: CollisionShapes,
    pub friction: f32,
    pub restitution: f32,
}

impl RigidBodyConstructionInfo {
    pub const fn new(collision_shape: CollisionShapes) -> Self {
        Self {
            collision_shape,
            friction: 0.5,
            restitution: 0.0,
            start_world_trans: Vec3A::ZERO,
        }
    }
}

#[derive(Clone)]
pub struct RigidBody {
    world_trans: Vec3A,
    shape: CollisionShapes,
    pub broadphase_handle: usize,
    pub friction: f32,
    pub restitution: f32,
    /// Cached shape breaking threshold (`angular_disc * 0.02`).
    /// Shapes never change after construction, so cache the disc math here
    /// Read it via [`get_contact_breaking_threshold`](Self::get_contact_breaking_threshold).
    contact_breaking_threshold: f32,
}

impl RigidBody {
    pub fn new(info: RigidBodyConstructionInfo) -> Self {
        // Shapes are immutable after construction, so the angular-disc
        // threshold never changes for this body. Cache it once.
        let aabb = info.collision_shape.get_aabb();
        let center = (aabb.min + aabb.max) * 0.5;
        let radius = (aabb.max - aabb.min).length() * 0.5;
        let contact_breaking_threshold = (radius + center.length()) * CONTACT_BREAKING_THRESHOLD;

        Self {
            world_trans: info.start_world_trans,
            broadphase_handle: 0,
            shape: info.collision_shape,
            friction: info.friction,
            restitution: info.restitution,
            contact_breaking_threshold,
        }
    }

    /// Cached breaking threshold for this body's shape (see field docs).
    /// Shapes are immutable, so this never changes after construction.
    #[inline]
    pub const fn get_contact_breaking_threshold(&self) -> f32 {
        self.contact_breaking_threshold
    }

    pub const fn get_world_trans(&self) -> Vec3A {
        self.world_trans
    }

    pub const fn get_collision_shape(&self) -> &CollisionShapes {
        &self.shape
    }
}
