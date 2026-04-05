use arrayvec::ArrayVec;
use glam::Vec3A;

use super::{contact_solver_info, solver_body::SolverBody, solver_constraint::SolverConstraint};
use crate::bullet::{
    collision::narrowphase::{
        manifold_point::ManifoldPoint, persistent_manifold::PersistentManifold,
    },
    dynamics::sphere_rigid_body::SphereRigidBody,
    linear_math::plane_space_1,
};

struct SpecialResolveInfo {
    pub num_special_collisions: u16,
    pub total_normal: Vec3A,
    pub total_dist: f32,
    pub restitution: f32,
    pub friction: f32,
}

impl SpecialResolveInfo {
    pub const DEFAULT: Self = Self {
        num_special_collisions: 0,
        total_normal: Vec3A::ZERO,
        total_dist: 0.0,
        restitution: 0.0,
        friction: 0.0,
    };

    fn add_special_collision(&mut self, cp: &ManifoldPoint, rel_pos: Vec3A) {
        self.num_special_collisions += 1;
        self.friction = cp.combined_friction;
        self.restitution = cp.combined_restitution;
        self.total_normal += cp.normal_world_on_b;
        self.total_dist += rel_pos.length();
    }
}

pub struct SeqImpulseConstraintSolver {
    solver_body: SolverBody,
    contact_constraint: Option<(SolverConstraint, SolverConstraint)>,
    special_resolve_info: SpecialResolveInfo,
}

impl Default for SeqImpulseConstraintSolver {
    fn default() -> Self {
        Self {
            solver_body: SolverBody::DEFAULT,
            contact_constraint: None,
            special_resolve_info: SpecialResolveInfo::DEFAULT,
        }
    }
}

impl SeqImpulseConstraintSolver {
    pub fn solve_group(
        &mut self,
        ball_obj: &mut SphereRigidBody,
        manifolds: &mut ArrayVec<PersistentManifold, 4>,
        time_step: f32,
    ) {
        self.solve_group_setup(ball_obj, manifolds, time_step);
        self.solve_group_iterations();
        self.solve_group_finish(ball_obj, time_step);
    }

    fn solve_group_setup(
        &mut self,
        ball_obj: &mut SphereRigidBody,
        manifolds: &mut ArrayVec<PersistentManifold, 4>,
        time_step: f32,
    ) {
        self.setup_solver_bodies(ball_obj, time_step);

        for manifold in manifolds.iter() {
            for cp in &manifold.point_cache {
                let rel_pos1 = cp.pos_world_on_a - ball_obj.get_world_trans();
                self.special_resolve_info
                    .add_special_collision(cp, rel_pos1);
            }
        }

        manifolds.clear();

        if self.special_resolve_info.num_special_collisions > 0 {
            self.convert_contact_special(ball_obj, time_step);
            self.special_resolve_info = SpecialResolveInfo::DEFAULT;
        }
    }

    fn setup_solver_bodies(&mut self, ball_obj: &mut SphereRigidBody, time_step: f32) {
        self.solver_body.update(ball_obj, time_step);
    }

    fn convert_contact_special(&mut self, body: &SphereRigidBody, time_step: f32) {
        let sri = &self.special_resolve_info;
        let num_collisions = f32::from(sri.num_special_collisions);
        let distance = sri.total_dist / num_collisions;
        let normal_world_on_b = sri.total_normal / num_collisions;

        let solver_body_a = &mut self.solver_body;

        let rel_pos1 = normal_world_on_b * -distance;
        let relaxation = contact_solver_info::SOR;

        let inv_time_step = 1.0 / time_step;
        let erp = contact_solver_info::ERP_2;

        let torque_axis_0 = rel_pos1.cross(normal_world_on_b);
        let angular_component_a = body.inv_inertia_local * torque_axis_0;

        let denom = {
            let vec = angular_component_a.cross(rel_pos1);
            body.inverse_mass + normal_world_on_b.dot(vec)
        };
        let jac_diag_ab_inv = relaxation / denom;

        let (contact_normal_1, rel_pos1_cross_normal) = (normal_world_on_b, torque_axis_0);

        let penetration = distance;

        let vel = body.get_vel_in_local_point(rel_pos1);
        let rel_vel = normal_world_on_b.dot(vel);

        let restitution = SolverConstraint::restitution_curve(rel_vel, sri.restitution).max(0.0);

        let external_force_impulse_a = solver_body_a.external_force_impulse;

        let rel_vel = contact_normal_1.dot(solver_body_a.lin_vel + external_force_impulse_a)
            + rel_pos1_cross_normal.dot(solver_body_a.ang_vel);

        let positional_error = if penetration > 0.0 {
            0.0
        } else {
            -penetration * erp * inv_time_step
        };

        let vel_error = restitution - rel_vel;

        let penetration_impulse = positional_error * jac_diag_ab_inv;
        let vel_impulse = vel_error * jac_diag_ab_inv;

        let (rhs, rhs_penetration) =
            if penetration > contact_solver_info::SPLIT_IMPULSE_PENETRATION_THRESHOLD {
                (penetration_impulse + vel_impulse, 0.0)
            } else {
                (vel_impulse, penetration_impulse)
            };

        let contact_constraint = SolverConstraint {
            angular_component_a,
            jac_diag_ab_inv,
            contact_normal_1,
            rel_pos1_cross_normal,
            rhs,
            rhs_penetration,
            friction: sri.friction,
            lower_limit: 0.0,
            upper_limit: 1e10,
            ..Default::default()
        };

        let vel = solver_body_a.get_vel_in_local_point_no_delta(rel_pos1);
        let rel_vel = normal_world_on_b.dot(vel);

        let mut lateral_friction_dir_1 = vel - normal_world_on_b * rel_vel;
        let lat_rel_vel = lateral_friction_dir_1.length_squared();

        if lat_rel_vel > f32::EPSILON {
            lateral_friction_dir_1 *= 1.0 / lat_rel_vel.sqrt();
        } else {
            lateral_friction_dir_1 = plane_space_1(normal_world_on_b);
        }

        // addFrictionConstraint
        let (contact_normal_1, rel_pos1_cross_normal, angular_component_a) = {
            let torque_axis = rel_pos1.cross(lateral_friction_dir_1);

            (
                lateral_friction_dir_1,
                torque_axis,
                body.inv_inertia_local * torque_axis,
            )
        };

        let denom = {
            let vec = angular_component_a.cross(rel_pos1);
            body.inverse_mass + lateral_friction_dir_1.dot(vec)
        };
        let jac_diag_ab_inv = relaxation / denom;

        let rel_vel = contact_normal_1.dot(solver_body_a.lin_vel + external_force_impulse_a)
            + rel_pos1_cross_normal.dot(solver_body_a.ang_vel);

        let vel_error = -rel_vel;
        let vel_impulse = vel_error * jac_diag_ab_inv;

        self.contact_constraint = Some((
            contact_constraint,
            SolverConstraint {
                contact_normal_1,
                rel_pos1_cross_normal,
                angular_component_a,
                jac_diag_ab_inv,
                rhs: vel_impulse,
                lower_limit: -sri.friction,
                upper_limit: sri.friction,
                friction: sri.friction,
                ..Default::default()
            },
        ));
    }

    fn solve_group_iterations(&mut self) {
        if let Some((contact, friction)) = self.contact_constraint.as_mut() {
            self.solver_body
                .solve_group_split_impulse_iterations(contact);

            for _ in 0..contact_solver_info::NUM_ITERATIONS {
                let least_squares_residual =
                    self.solver_body.solve_single_iteration(contact, friction);
                if least_squares_residual == 0.0 {
                    break;
                }
            }
        }
    }

    fn solve_group_finish(&mut self, body: &mut SphereRigidBody, time_step: f32) {
        self.solver_body.lin_vel += self.solver_body.delta_lin_vel;
        self.solver_body.ang_vel += self.solver_body.delta_ang_vel;

        body.set_lin_vel(self.solver_body.lin_vel + self.solver_body.external_force_impulse);
        body.set_ang_vel(self.solver_body.ang_vel);

        body.set_world_trans(body.get_world_trans() + self.solver_body.push_vel * time_step);

        self.contact_constraint = None;
    }
}
