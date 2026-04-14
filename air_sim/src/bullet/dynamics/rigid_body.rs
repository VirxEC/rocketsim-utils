use glam::{Affine3A, Mat3A, Vec3A};

use crate::bullet::linear_math::transform_util::integrate_trans;

pub enum RigidBodyFlags {
    DisableWorldGravity = 1,
}

pub struct RigidBodyConstructionInfo {
    pub mass: f32,
    pub start_world_trans: Affine3A,
    pub local_inertia: Vec3A,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub friction: f32,
    pub restitution: f32,
}

impl RigidBodyConstructionInfo {
    pub const fn new(mass: f32) -> Self {
        Self {
            mass,
            local_inertia: Vec3A::ZERO,
            linear_damping: 0.0,
            angular_damping: 0.0,
            friction: 0.5,
            restitution: 0.0,
            start_world_trans: Affine3A::IDENTITY,
        }
    }
}

pub struct RigidBody {
    world_trans: Affine3A,
    rb_flags: u8,

    pub interp_world_trans: Affine3A,
    pub companion_id: Option<usize>,
    /// The index of this object in `CollisionWorld`
    pub world_array_idx: usize,

    pub inv_inertia_tensor_world: Mat3A,
    pub lin_vel: Vec3A,
    pub ang_vel: Vec3A,
    pub inverse_mass: f32,
    pub gravity: Vec3A,
    pub gravity_acceleration: Vec3A,
    pub inv_inertia_local: Vec3A,
    pub total_force: Vec3A,
    pub total_torque: Vec3A,
    pub linear_damping: f32,
    pub angular_damping: f32,
}

impl RigidBody {
    pub fn new(info: RigidBodyConstructionInfo) -> Self {
        let inverse_mass = if info.mass == 0.0 {
            0.0
        } else {
            1.0 / info.mass
        };

        let linear_damping = info.linear_damping.clamp(0.0, 1.0);
        let angular_damping = info.angular_damping.clamp(0.0, 1.0);

        let inv_inertia_local = Vec3A::select(
            info.local_inertia.cmpeq(Vec3A::ZERO),
            Vec3A::ZERO,
            1.0 / info.local_inertia,
        );

        let inv_inertia_tensor_world =
            Self::get_inertia_tensor(info.start_world_trans.matrix3, inv_inertia_local);

        Self {
            world_trans: info.start_world_trans,
            interp_world_trans: info.start_world_trans,
            companion_id: None,
            world_array_idx: 0,

            inv_inertia_tensor_world,
            lin_vel: Vec3A::ZERO,
            ang_vel: Vec3A::ZERO,
            inverse_mass,
            gravity: Vec3A::ZERO,
            gravity_acceleration: Vec3A::ZERO,
            inv_inertia_local,
            total_force: Vec3A::ZERO,
            total_torque: Vec3A::ZERO,
            linear_damping,
            angular_damping,
            rb_flags: 0,
        }
    }

    pub const fn set_world_trans(&mut self, world_trans: Affine3A) {
        self.world_trans = world_trans;
    }

    pub const fn get_world_trans(&self) -> &Affine3A {
        &self.world_trans
    }

    pub const fn get_rb_flags(&self) -> u8 {
        self.rb_flags
    }

    pub fn set_gravity(&mut self, acceleration: Vec3A) {
        if self.inverse_mass != 0.0 {
            self.gravity = acceleration * (1.0 / self.inverse_mass);
        }

        self.gravity_acceleration = acceleration;
    }

    pub fn set_lin_vel(&mut self, lin_vel: Vec3A) {
        debug_assert!(!lin_vel.is_nan());
        self.lin_vel = lin_vel;
    }

    pub fn set_ang_vel(&mut self, ang_vel: Vec3A) {
        debug_assert!(!ang_vel.is_nan());
        self.ang_vel = ang_vel;
    }

    fn get_inertia_tensor(world_mat: Mat3A, inv_inertia_local: Vec3A) -> Mat3A {
        let mut scaled_mat = world_mat.transpose();
        scaled_mat.x_axis *= inv_inertia_local;
        scaled_mat.y_axis *= inv_inertia_local;
        scaled_mat.z_axis *= inv_inertia_local;

        world_mat * scaled_mat
    }

    pub fn update_inertia_tensor(&mut self) {
        self.inv_inertia_tensor_world =
            Self::get_inertia_tensor(self.get_world_trans().matrix3, self.inv_inertia_local);
    }

    pub fn apply_torque(&mut self, torque: Vec3A) {
        debug_assert!(!torque.is_nan());
        self.total_torque += torque;
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
        self.ang_vel *= (1.0 - self.angular_damping).powf(time_step);
    }

    pub fn predict_integration_trans(&self, time_step: f32) -> Affine3A {
        let mut trans = *self.get_world_trans();
        integrate_trans(&mut trans, self.lin_vel, self.ang_vel, time_step);
        trans
    }

    pub fn set_center_of_mass_trans(&mut self, xform: Affine3A) {
        self.interp_world_trans = xform;
        self.set_world_trans(xform);

        self.update_inertia_tensor();
    }

    pub const fn clear_forces(&mut self) {
        self.total_force = Vec3A::ZERO;
        self.total_torque = Vec3A::ZERO;
    }

    pub const fn get_forward_vector(&self) -> Vec3A {
        self.get_world_trans().matrix3.x_axis
    }

    pub fn get_forward_speed(&self) -> f32 {
        self.lin_vel.dot(self.get_forward_vector())
    }
}
