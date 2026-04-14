use super::{contact_solver_info, solver_body::SolverBody};
use crate::bullet::{
    dynamics::rigid_body::RigidBody, linear_math::transform_util::integrate_trans,
};

#[derive(Default)]
pub struct SeqImpulseConstraintSolver {
    tmp_solver_body_pool: Vec<SolverBody>,
}

impl SeqImpulseConstraintSolver {
    pub fn solve_group(&mut self, collision_objs: &mut [RigidBody], time_step: f32) {
        self.setup_solver_bodies(collision_objs, time_step);
        self.solve_group_finish(collision_objs, time_step);
    }

    fn setup_solver_bodies(&mut self, collision_objs: &mut [RigidBody], time_step: f32) {
        self.tmp_solver_body_pool.reserve(collision_objs.len());

        for rb in collision_objs {
            let solver_body_id = self.tmp_solver_body_pool.len();
            rb.companion_id = Some(solver_body_id);

            self.tmp_solver_body_pool
                .push(SolverBody::new(rb, time_step));
        }
    }

    fn solve_group_finish(&mut self, collision_objs: &mut [RigidBody], time_step: f32) {
        for solver in &mut self.tmp_solver_body_pool {
            let body = &mut collision_objs[solver.original_body];

            solver.lin_vel += solver.delta_lin_vel;
            solver.ang_vel += solver.delta_ang_vel;

            if solver.push_vel.length_squared() != 0.0 || solver.turn_vel.length_squared() != 0.0 {
                integrate_trans(
                    &mut solver.world_trans,
                    solver.push_vel,
                    solver.turn_vel * contact_solver_info::SPLIT_IMPULSE_TURN_ERP,
                    time_step,
                );
            }

            body.set_lin_vel(solver.lin_vel + solver.external_force_impulse);
            body.set_ang_vel(solver.ang_vel + solver.external_torque_impulse);

            body.set_world_trans(solver.world_trans);
        }

        self.tmp_solver_body_pool.clear();
    }
}
