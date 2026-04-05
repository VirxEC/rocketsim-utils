use glam::Vec3A;

use crate::bullet::collision::shapes::sphere_shape::SphereShape;

pub struct SphereRigidBodyConstructionInfo {
    pub mass: f32,
    pub start_world_trans: Vec3A,
    pub collision_shape: SphereShape,
    pub local_inertia: Vec3A,
    pub linear_damping: f32,
    pub friction: f32,
    pub restitution: f32,
}

impl SphereRigidBodyConstructionInfo {
    pub const fn new(mass: f32, collision_shape: SphereShape) -> Self {
        Self {
            mass,
            collision_shape,
            local_inertia: Vec3A::ZERO,
            linear_damping: 0.0,
            friction: 0.5,
            restitution: 0.0,
            start_world_trans: Vec3A::ZERO,
        }
    }
}

pub struct SphereRigidBody {
    world_trans: Vec3A,
    shape: SphereShape,
    pub interp_world_trans: Vec3A,
    pub friction: f32,
    pub restitution: f32,
    pub lin_vel: Vec3A,
    pub ang_vel: Vec3A,
    pub inverse_mass: f32,
    pub gravity: Vec3A,
    pub inv_inertia_local: Vec3A,
    pub total_force: Vec3A,
    pub linear_damping: f32,
    pub inv_mass: Vec3A,
}

impl SphereRigidBody {
    pub fn new(info: SphereRigidBodyConstructionInfo) -> Self {
        let inverse_mass = if info.mass == 0.0 {
            0.0
        } else {
            1.0 / info.mass
        };

        let linear_damping = info.linear_damping.clamp(0.0, 1.0);

        let inv_inertia_local = Vec3A::select(
            info.local_inertia.cmpeq(Vec3A::ZERO),
            Vec3A::ZERO,
            1.0 / info.local_inertia,
        );

        Self {
            world_trans: info.start_world_trans,
            interp_world_trans: info.start_world_trans,
            shape: info.collision_shape,
            friction: info.friction,
            restitution: info.restitution,
            lin_vel: Vec3A::ZERO,
            ang_vel: Vec3A::ZERO,
            inverse_mass,
            gravity: Vec3A::ZERO,
            inv_inertia_local,
            total_force: Vec3A::ZERO,
            linear_damping,
            inv_mass: Vec3A::splat(inverse_mass),
        }
    }

    pub const fn set_world_trans(&mut self, world_trans: Vec3A) {
        self.world_trans = world_trans;
    }

    pub const fn get_world_trans(&self) -> Vec3A {
        self.world_trans
    }

    pub const fn get_collision_shape(&self) -> &SphereShape {
        &self.shape
    }

    pub fn set_gravity(&mut self, acceleration: Vec3A) {
        debug_assert_ne!(self.inverse_mass, 0.0);
        self.gravity = acceleration * (1.0 / self.inverse_mass);
    }

    pub fn set_lin_vel(&mut self, lin_vel: Vec3A) {
        debug_assert!(!lin_vel.is_nan());
        self.lin_vel = lin_vel;
    }

    pub fn set_ang_vel(&mut self, ang_vel: Vec3A) {
        debug_assert!(!ang_vel.is_nan());
        self.ang_vel = ang_vel;
    }

    pub fn apply_central_impulse(&mut self, impulse: Vec3A) {
        debug_assert!(!impulse.is_nan());
        self.lin_vel += impulse * self.inverse_mass;
    }

    pub fn apply_central_force(&mut self, force: Vec3A) {
        debug_assert!(!force.is_nan());
        self.total_force += force;
    }

    pub fn apply_gravity(&mut self) {
        self.apply_central_force(self.gravity);
    }

    pub fn apply_damping(&mut self, time_step: f32) {
        self.lin_vel *= (1.0 - self.linear_damping).powf(time_step);
    }

    pub fn predict_integration_trans(&self, time_step: f32) -> Vec3A {
        self.world_trans + self.lin_vel * time_step
    }

    pub fn set_center_of_mass_trans(&mut self, xform: Vec3A) {
        self.interp_world_trans = xform;
        self.set_world_trans(xform);
    }

    pub fn get_vel_in_local_point(&self, rel_pos: Vec3A) -> Vec3A {
        self.lin_vel + self.ang_vel.cross(rel_pos)
    }

    pub const fn clear_forces(&mut self) {
        self.total_force = Vec3A::ZERO;
    }

    pub fn get_mass(&self) -> f32 {
        if self.inverse_mass == 0.0 {
            0.0
        } else {
            1.0 / self.inverse_mass
        }
    }
}
