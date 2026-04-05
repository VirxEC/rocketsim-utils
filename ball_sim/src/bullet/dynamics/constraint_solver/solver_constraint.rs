use glam::Vec3A;

use super::solver_body::SolverBody;
use crate::bullet::dynamics::constraint_solver::contact_solver_info;

fn bullet_dot(vec0: Vec3A, vec1: Vec3A) -> f32 {
    let result = vec0 * vec1;
    result.x + (result.y + result.z)
}

#[derive(Default)]
pub struct SolverConstraint {
    pub rel_pos1_cross_normal: Vec3A,
    pub contact_normal_1: Vec3A,
    pub angular_component_a: Vec3A,
    pub applied_push_impulse: f32,
    pub applied_impulse: f32,
    pub friction: f32,
    pub jac_diag_ab_inv: f32,
    pub rhs: f32,
    pub lower_limit: f32,
    pub upper_limit: f32,
    pub rhs_penetration: f32,
}

impl SolverConstraint {
    pub fn restitution_curve(rel_vel: f32, restitution: f32) -> f32 {
        if rel_vel.abs() < contact_solver_info::RESTITUTION_VELOCITY_THRESHOLD {
            0.0
        } else {
            restitution * -rel_vel
        }
    }

    pub fn resolve_single_constraint_row_generic(&mut self, body_a: &mut SolverBody) -> f32 {
        let mut delta_impulse = self.rhs;

        let delta_vel_1_dot_n = bullet_dot(self.contact_normal_1, body_a.delta_lin_vel)
            + bullet_dot(self.rel_pos1_cross_normal, body_a.delta_ang_vel);

        delta_impulse -= delta_vel_1_dot_n * self.jac_diag_ab_inv;

        let sum = self.applied_impulse + delta_impulse;
        if sum < self.lower_limit {
            delta_impulse = self.lower_limit - self.applied_impulse;
            self.applied_impulse = self.lower_limit;
        } else if sum > self.upper_limit {
            delta_impulse = self.upper_limit - self.applied_impulse;
            self.applied_impulse = self.upper_limit;
        } else {
            self.applied_impulse = sum;
        }

        body_a.delta_lin_vel += self.contact_normal_1 * body_a.inv_mass * delta_impulse;
        body_a.delta_ang_vel += self.angular_component_a * delta_impulse;

        delta_impulse / self.jac_diag_ab_inv
    }

    pub fn resolve_single_constraint_row_lower_limit(&mut self, body_a: &mut SolverBody) -> f32 {
        let mut delta_impulse = self.rhs;

        let delta_vel_1_dot_n = bullet_dot(self.contact_normal_1, body_a.delta_lin_vel)
            + bullet_dot(self.rel_pos1_cross_normal, body_a.delta_ang_vel);

        delta_impulse -= delta_vel_1_dot_n * self.jac_diag_ab_inv;

        let sum = self.applied_impulse + delta_impulse;
        if sum < self.lower_limit {
            delta_impulse = self.lower_limit - self.applied_impulse;
            self.applied_impulse = self.lower_limit;
        } else {
            self.applied_impulse = sum;
        }

        body_a.delta_lin_vel += self.contact_normal_1 * body_a.inv_mass * delta_impulse;
        body_a.delta_ang_vel += self.angular_component_a * delta_impulse;

        delta_impulse / self.jac_diag_ab_inv
    }

    pub fn resolve_split_penetration_impulse(&mut self, body_a: &mut SolverBody) -> f32 {
        if self.rhs_penetration == 0.0 {
            return 0.0;
        }

        let mut delta_impulse = self.rhs_penetration;

        let delta_vel_1_dot_n = bullet_dot(self.contact_normal_1, body_a.push_vel)
            + bullet_dot(self.rel_pos1_cross_normal, body_a.turn_vel);

        delta_impulse -= delta_vel_1_dot_n * self.jac_diag_ab_inv;

        let sum = self.applied_push_impulse + delta_impulse;
        if sum < self.lower_limit {
            delta_impulse = self.lower_limit - self.applied_push_impulse;
            self.applied_push_impulse = self.lower_limit;
        } else {
            self.applied_push_impulse = sum;
        }

        body_a.push_vel += self.contact_normal_1 * body_a.inv_mass * delta_impulse;
        body_a.turn_vel += self.angular_component_a * delta_impulse;

        delta_impulse / self.jac_diag_ab_inv
    }
}
