use glam::{Affine3A, Mat3A, Vec3A};

use crate::bullet::linear_math::transform_util::integrate_trans;

pub struct RigidBody {
    world_trans: Affine3A,
    pub inv_inertia_tensor_world: Mat3A,
    pub lin_vel: Vec3A,
    pub ang_vel: Vec3A,
    pub inverse_mass: f32,
    pub gravity: Vec3A,
    pub inv_inertia_local: Vec3A,
    pub total_force: Vec3A,
    pub total_torque: Vec3A,
}

impl RigidBody {
    pub fn new(mass: f32, local_inertia: Vec3A) -> Self {
        let inverse_mass = 1.0 / mass;

        let inv_inertia_local = Vec3A::select(
            local_inertia.cmpeq(Vec3A::ZERO),
            Vec3A::ZERO,
            1.0 / local_inertia,
        );

        let inv_inertia_tensor_world = Self::get_inertia_tensor(Mat3A::IDENTITY, inv_inertia_local);

        Self {
            world_trans: Affine3A::IDENTITY,
            inv_inertia_tensor_world,
            lin_vel: Vec3A::ZERO,
            ang_vel: Vec3A::ZERO,
            inverse_mass,
            gravity: Vec3A::ZERO,
            inv_inertia_local,
            total_force: Vec3A::ZERO,
            total_torque: Vec3A::ZERO,
        }
    }

    pub const fn set_world_trans(&mut self, world_trans: Affine3A) {
        self.world_trans = world_trans;
    }

    pub const fn get_world_trans(&self) -> &Affine3A {
        &self.world_trans
    }

    pub fn set_gravity(&mut self, acceleration: Vec3A) {
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

    pub fn integration_trans(&mut self, time_step: f32) {
        integrate_trans(&mut self.world_trans, self.lin_vel, self.ang_vel, time_step);
        self.update_inertia_tensor();
    }

    pub fn set_center_of_mass_trans(&mut self, xform: Affine3A) {
        self.world_trans = xform;
        self.update_inertia_tensor();
    }

    pub const fn clear_forces(&mut self) {
        self.total_force = Vec3A::ZERO;
        self.total_torque = Vec3A::ZERO;
    }

    pub fn get_forward_speed(&self) -> f32 {
        self.lin_vel.dot(self.get_world_trans().matrix3.x_axis)
    }
}
