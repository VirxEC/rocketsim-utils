use glam::Vec3A;

use crate::bullet::collision::shapes::collision_shape::CollisionShapes;

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
}

impl RigidBody {
    pub fn new(info: RigidBodyConstructionInfo) -> Self {
        Self {
            world_trans: info.start_world_trans,
            broadphase_handle: 0,
            shape: info.collision_shape,
            friction: info.friction,
            restitution: info.restitution,
        }
    }

    pub const fn get_world_trans(&self) -> Vec3A {
        self.world_trans
    }

    pub const fn get_collision_shape(&self) -> &CollisionShapes {
        &self.shape
    }
}
