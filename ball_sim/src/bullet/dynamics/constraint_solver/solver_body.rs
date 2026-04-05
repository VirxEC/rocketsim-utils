use glam::Vec3A;

use crate::bullet::dynamics::{
    constraint_solver::{contact_solver_info, solver_constraint::SolverConstraint},
    sphere_rigid_body::SphereRigidBody,
};

pub struct SolverBody {
    pub delta_lin_vel: Vec3A,
    pub delta_ang_vel: Vec3A,
    pub inv_mass: Vec3A,
    pub push_vel: Vec3A,
    pub turn_vel: Vec3A,
    pub lin_vel: Vec3A,
    pub ang_vel: Vec3A,
    pub external_force_impulse: Vec3A,
}

impl SolverBody {
    pub const DEFAULT: Self = Self {
        delta_lin_vel: Vec3A::ZERO,
        delta_ang_vel: Vec3A::ZERO,
        inv_mass: Vec3A::ZERO,
        push_vel: Vec3A::ZERO,
        turn_vel: Vec3A::ZERO,
        lin_vel: Vec3A::ZERO,
        ang_vel: Vec3A::ZERO,
        external_force_impulse: Vec3A::ZERO,
    };

    pub fn get_vel_in_local_point_no_delta(&self, rel_pos: Vec3A) -> Vec3A {
        self.lin_vel + self.external_force_impulse + self.ang_vel.cross(rel_pos)
    }

    pub fn update(&mut self, rb: &mut SphereRigidBody, time_step: f32) {
        self.delta_lin_vel = Vec3A::ZERO;
        self.delta_ang_vel = Vec3A::ZERO;
        self.inv_mass = rb.inv_mass;
        self.push_vel = Vec3A::ZERO;
        self.turn_vel = Vec3A::ZERO;
        self.lin_vel = rb.lin_vel;
        self.ang_vel = rb.ang_vel;
        self.external_force_impulse = rb.total_force * rb.inverse_mass * time_step;
    }

    pub fn solve_group_split_impulse_iterations(&mut self, contact: &mut SolverConstraint) {
        for _ in 0..contact_solver_info::NUM_ITERATIONS {
            let residual = contact.resolve_split_penetration_impulse(self);
            if residual * residual == 0.0 {
                continue;
            }
        }
    }

    pub fn solve_single_iteration(
        &mut self,
        contact: &mut SolverConstraint,
        friction: &mut SolverConstraint,
    ) -> f32 {
        let mut least_squares_residual = 0.0;

        let residual = contact.resolve_single_constraint_row_lower_limit(self);
        least_squares_residual = (residual * residual).max(least_squares_residual);

        if contact.applied_impulse > 0.0 {
            let limit = friction.friction * contact.applied_impulse;
            friction.lower_limit = -limit;
            friction.upper_limit = limit;

            let residual = friction.resolve_single_constraint_row_generic(self);
            least_squares_residual = (residual * residual).max(least_squares_residual);
        }

        least_squares_residual
    }
}
