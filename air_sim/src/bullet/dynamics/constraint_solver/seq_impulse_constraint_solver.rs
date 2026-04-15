use super::{contact_solver_info, solver_body::SolverBody};
use crate::bullet::{
    dynamics::rigid_body::RigidBody, linear_math::transform_util::integrate_trans,
};

#[derive(Default)]
pub struct SeqImpulseConstraintSolver {
    solver_body: SolverBody,
}

impl SeqImpulseConstraintSolver {
    pub fn solve_group(&mut self, collision_obj: &mut RigidBody, time_step: f32) {
        self.setup_solver_bodies(collision_obj, time_step);
        self.solve_group_finish(collision_obj, time_step);
    }

    fn setup_solver_bodies(&mut self, rb: &mut RigidBody, time_step: f32) {
        self.solver_body = SolverBody::new(rb, time_step);
    }

    fn solve_group_finish(&mut self, body: &mut RigidBody, time_step: f32) {
        self.solver_body.lin_vel += self.solver_body.delta_lin_vel;
        self.solver_body.ang_vel += self.solver_body.delta_ang_vel;

        if self.solver_body.push_vel.length_squared() != 0.0
            || self.solver_body.turn_vel.length_squared() != 0.0
        {
            integrate_trans(
                &mut self.solver_body.world_trans,
                self.solver_body.push_vel,
                self.solver_body.turn_vel * contact_solver_info::SPLIT_IMPULSE_TURN_ERP,
                time_step,
            );
        }

        body.set_lin_vel(self.solver_body.lin_vel + self.solver_body.external_force_impulse);
        body.set_ang_vel(self.solver_body.ang_vel + self.solver_body.external_torque_impulse);

        body.set_world_trans(self.solver_body.world_trans);
    }
}
