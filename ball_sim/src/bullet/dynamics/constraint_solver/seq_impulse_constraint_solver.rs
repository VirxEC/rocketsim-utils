use glam::Vec3A;

use super::{contact_solver_info, solver_body::SolverBody, solver_constraint::SolverConstraint};
use crate::bullet::{
    collision::narrowphase::{
        manifold_point::ManifoldPoint,
        persistent_manifold::{PersistentManifold, pair_key},
    },
    dynamics::sphere_rigid_body::SphereRigidBody,
};

const BALL_SOLVER_BODY_ID: usize = 0;
const FIXED_SOLVER_BODY_ID: usize = 1;

/// Hold one flagged sample for the per-body accumulator.
/// Store the adjusted normal. Store the lever length. Store materials.
#[derive(Clone, Copy, Debug)]
struct SpecialSample {
    normal_world_on_b: Vec3A,
    lever_len: f32,
    friction: f32,
    restitution: f32,
}

/// Keep first-seen order in the accumulator vector.
#[derive(Clone, Copy, Debug, Default)]
struct SpecialAccumulator {
    total_normal: Vec3A,
    total_lever_len: f32,
    total_friction: f32,
    total_restitution: f32,
    count: u32,
}

fn accumulate_special_sample(accumulator: &mut Option<SpecialAccumulator>, sample: SpecialSample) {
    if let Some(acc) = accumulator {
        acc.total_normal += sample.normal_world_on_b;
        acc.total_lever_len += sample.lever_len;
        acc.total_friction += sample.friction;
        acc.total_restitution += sample.restitution;
        acc.count += 1;
        return;
    }

    *accumulator = Some(SpecialAccumulator {
        total_normal: sample.normal_world_on_b,
        total_lever_len: sample.lever_len,
        total_friction: sample.friction,
        total_restitution: sample.restitution,
        count: 1,
    });
}

#[derive(Clone, Debug)]
pub struct SeqImpulseConstraintSolver {
    solver_bodies: [SolverBody; 2],
    tmp_solver_contact_constraint_pool: Vec<SolverConstraint>,
    tmp_solver_contact_friction_constraint_pool: Vec<SolverConstraint>,
    least_squares_residual: f32,
    tmp_special_accumulator: Option<SpecialAccumulator>,
    tmp_split_should_run: Vec<u8>,
}

impl Default for SeqImpulseConstraintSolver {
    fn default() -> Self {
        Self {
            solver_bodies: [SolverBody::DEFAULT, SolverBody::DEFAULT],
            tmp_solver_contact_constraint_pool: Vec::new(),
            tmp_solver_contact_friction_constraint_pool: Vec::new(),
            least_squares_residual: 0.0,
            tmp_special_accumulator: None,
            tmp_split_should_run: Vec::new(),
        }
    }
}

impl SeqImpulseConstraintSolver {
    pub fn solve_group(
        &mut self,
        ball_obj: &mut SphereRigidBody,
        manifolds: &mut [PersistentManifold],
        active_manifold_idcs: &mut Vec<usize>,
        time_step: f32,
    ) {
        self.solve_group_setup(ball_obj, manifolds, active_manifold_idcs, time_step);
        self.solve_group_iterations();
        self.solve_group_finish(ball_obj, time_step);
    }

    fn solve_group_setup(
        &mut self,
        ball_obj: &mut SphereRigidBody,
        manifolds: &mut [PersistentManifold],
        active_manifold_idcs: &mut Vec<usize>,
        time_step: f32,
    ) {
        self.setup_solver_bodies(ball_obj);
        self.tmp_special_accumulator = None;

        let ball_trans = ball_obj.get_world_trans();
        for &manifold_idx in active_manifold_idcs.iter() {
            let manifold = &mut manifolds[manifold_idx];
            debug_assert_eq!(manifold.body0_idx, 0);
            debug_assert_eq!(
                manifold.pair_key,
                pair_key(manifold.body0_idx, manifold.body1_idx)
            );

            for cp in &mut manifold.point_cache {
                assert!(cp.distance_1 <= manifold.contact_processing_threshold);

                let rel_pos1 = cp.pos_world_on_a - ball_trans;
                let rel_pos2 = Vec3A::ZERO;

                debug_assert!(cp.is_special);
                if cp.distance_1 < 0.0 {
                    let [solver_body_a, solver_body_b] = unsafe {
                        self.solver_bodies
                            .get_disjoint_unchecked_mut([BALL_SOLVER_BODY_ID, FIXED_SOLVER_BODY_ID])
                    };

                    self.tmp_solver_contact_constraint_pool.push(
                        SolverConstraint::get_split_only_contact_constraint(
                            (BALL_SOLVER_BODY_ID, FIXED_SOLVER_BODY_ID),
                            (solver_body_a, solver_body_b),
                            (Some(ball_obj), None),
                            (rel_pos1, rel_pos2),
                            cp,
                            time_step,
                        ),
                    );
                }

                accumulate_special_sample(
                    &mut self.tmp_special_accumulator,
                    SpecialSample {
                        normal_world_on_b: cp.normal_world_on_b,
                        lever_len: rel_pos1.length(),
                        friction: cp.combined_friction,
                        restitution: cp.combined_restitution,
                    },
                );
            }
        }

        // Persistent manifolds keep their points and warmstart impulses for the next tick.
        active_manifold_idcs.clear();

        if self.tmp_special_accumulator.is_some() {
            self.emit_special_synthetics(ball_obj, time_step);
        }
    }

    fn setup_solver_bodies(&mut self, ball_obj: &SphereRigidBody) {
        self.solver_bodies[BALL_SOLVER_BODY_ID] = SolverBody::new(ball_obj);
        self.solver_bodies[FIXED_SOLVER_BODY_ID] = SolverBody::DEFAULT;
    }

    /// Use plain means. Normalize the mean normal. Use a radial lever.
    /// Use distance `0.0` from the evidence-backed template. Keep first-seen order.
    fn emit_special_synthetics(&mut self, ball_obj: &SphereRigidBody, time_step: f32) {
        if let Some(acc) = self.tmp_special_accumulator {
            self.push_special_synthetic(ball_obj, &acc, time_step);
        }
    }

    fn push_special_synthetic(
        &mut self,
        ball_obj: &SphereRigidBody,
        acc: &SpecialAccumulator,
        time_step: f32,
    ) {
        debug_assert!(acc.count > 0);
        let num_samples = acc.count as f32;
        let mean_normal = (acc.total_normal / num_samples).normalize_or_zero();
        let mean_lever_len = acc.total_lever_len / num_samples;
        let mean_friction = acc.total_friction / num_samples;
        let mean_restitution = acc.total_restitution / num_samples;

        let [solver_body_a, solver_body_b] = unsafe {
            self.solver_bodies
                .get_disjoint_unchecked_mut([BALL_SOLVER_BODY_ID, FIXED_SOLVER_BODY_ID])
        };

        // Use a radial lever. Use template distance `0.0`.
        let rel_pos1 = mean_normal * -mean_lever_len;
        let rel_pos2 = Vec3A::ZERO;
        let synthetic_point = ManifoldPoint {
            normal_world_on_b: mean_normal,
            combined_friction: mean_friction,
            combined_restitution: mean_restitution,
            distance_1: 0.0,
            applied_impulse: 0.0,
            ..Default::default()
        };

        let friction_idx = self.tmp_solver_contact_constraint_pool.len();

        self.tmp_solver_contact_constraint_pool
            .push(SolverConstraint::get_contact_constraint(
                (BALL_SOLVER_BODY_ID, FIXED_SOLVER_BODY_ID),
                (solver_body_a, solver_body_b),
                (Some(ball_obj), None),
                (rel_pos1, rel_pos2),
                &synthetic_point,
                friction_idx,
                time_step,
            ));

        let lateral_friction_dir_1 =
            synthetic_point.calc_lat_friction_dir(solver_body_a, solver_body_b, rel_pos1, rel_pos2);

        self.tmp_solver_contact_friction_constraint_pool.push(
            SolverConstraint::get_friction_constraint(
                (BALL_SOLVER_BODY_ID, FIXED_SOLVER_BODY_ID),
                (solver_body_a, solver_body_b),
                (Some(ball_obj), None),
                (rel_pos1, rel_pos2),
                synthetic_point.combined_friction,
                lateral_friction_dir_1,
                friction_idx,
            ),
        );
    }

    fn solve_group_split_impulse_iterations_one_dynamic(&mut self) {
        let row_count = self.tmp_solver_contact_constraint_pool.len();
        self.tmp_split_should_run.clear();
        self.tmp_split_should_run.resize(row_count, 1);
        let mut remaining = row_count;

        for _ in 0..contact_solver_info::NUM_ITERATIONS {
            if remaining == 0 {
                break;
            }
            for (i, contact) in self
                .tmp_solver_contact_constraint_pool
                .iter_mut()
                .enumerate()
            {
                if self.tmp_split_should_run[i] == 0 {
                    continue;
                }

                debug_assert_ne!(contact.solver_body_id_a, contact.solver_body_id_b);
                let body_a = &mut self.solver_bodies[contact.solver_body_id_a];
                let residual = contact.resolve_split_penetration_impulse_one_dynamic(body_a);
                if residual * residual == 0.0 {
                    self.tmp_split_should_run[i] = 0;
                    remaining -= 1;
                }
            }
        }
    }

    fn solve_single_iteration_one_dynamic(&mut self) -> f32 {
        let Some(contact) = self.tmp_solver_contact_constraint_pool.last_mut() else {
            return 0.0;
        };
        debug_assert!(!contact.is_split_only);
        let body_a = &mut self.solver_bodies[contact.solver_body_id_a];
        let residual = contact.resolve_single_constraint_row_lower_limit_one_dynamic(body_a);
        let mut least_squares_residual = residual * residual;

        for contact in &mut self.tmp_solver_contact_friction_constraint_pool {
            let total_impulse =
                self.tmp_solver_contact_constraint_pool[contact.friction_idx].applied_impulse;
            if total_impulse <= 0.0 {
                continue;
            }

            let limit = contact.friction * total_impulse;
            contact.lower_limit = -limit;
            contact.upper_limit = limit;

            debug_assert_ne!(contact.solver_body_id_a, contact.solver_body_id_b);
            let body_a = &mut self.solver_bodies[contact.solver_body_id_a];
            let residual = contact.resolve_single_constraint_row_generic_one_dynamic(body_a);
            least_squares_residual = (residual * residual).max(least_squares_residual);
        }

        least_squares_residual
    }

    fn solve_group_iterations(&mut self) {
        self.solve_group_split_impulse_iterations_one_dynamic();

        for _ in 0..contact_solver_info::NUM_ITERATIONS {
            self.least_squares_residual = self.solve_single_iteration_one_dynamic();
            if self.least_squares_residual == 0.0 {
                break;
            }
        }
    }

    fn solve_group_finish(&mut self, body: &mut SphereRigidBody, time_step: f32) {
        let solver = &mut self.solver_bodies[BALL_SOLVER_BODY_ID];
        solver.lin_vel += solver.delta_lin_vel;
        solver.ang_vel += solver.delta_ang_vel;

        if solver.push_vel.length_squared() != 0.0 || solver.turn_vel.length_squared() != 0.0 {
            body.set_world_trans(body.get_world_trans() + solver.push_vel * time_step);
        }

        body.set_lin_vel(solver.lin_vel + solver.external_force_impulse);
        body.set_ang_vel(solver.ang_vel + solver.external_torque_impulse);

        self.tmp_solver_contact_constraint_pool.clear();
        self.tmp_solver_contact_friction_constraint_pool.clear();
        self.tmp_split_should_run.clear();
    }
}
