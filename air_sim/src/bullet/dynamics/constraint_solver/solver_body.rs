use glam::{Affine3A, Vec3A};

use crate::bullet::dynamics::rigid_body::RigidBody;

#[derive(Default)]
pub struct SolverBody {
    pub world_trans: Affine3A,
    pub delta_lin_vel: Vec3A,
    pub delta_ang_vel: Vec3A,
    pub push_vel: Vec3A,
    pub turn_vel: Vec3A,
    pub lin_vel: Vec3A,
    pub ang_vel: Vec3A,
    pub external_force_impulse: Vec3A,
    pub external_torque_impulse: Vec3A,
}

impl SolverBody {
    pub fn new(rb: &RigidBody, time_step: f32) -> Self {
        Self {
            world_trans: *rb.get_world_trans(),
            delta_lin_vel: Vec3A::ZERO,
            delta_ang_vel: Vec3A::ZERO,
            push_vel: Vec3A::ZERO,
            turn_vel: Vec3A::ZERO,
            lin_vel: rb.lin_vel,
            ang_vel: rb.ang_vel,
            external_force_impulse: rb.total_force * rb.inverse_mass * time_step,
            external_torque_impulse: rb.inv_inertia_tensor_world * rb.total_torque * time_step,
        }
    }
}
