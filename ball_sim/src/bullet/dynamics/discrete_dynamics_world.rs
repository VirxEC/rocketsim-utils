use glam::Vec3A;

use super::{
    constraint_solver::seq_impulse_constraint_solver::SeqImpulseConstraintSolver,
    rigid_body::RigidBody,
};
use crate::{
    ArenaContactTracker,
    bullet::{
        collision::{broadphase::GridBroadphase, dispatch::collision_world::CollisionWorld},
        dynamics::sphere_rigid_body::SphereRigidBody,
    },
};

#[derive(Clone)]
pub struct DiscreteDynamicsWorld {
    collision_world: CollisionWorld,
    solver: SeqImpulseConstraintSolver,
}

impl DiscreteDynamicsWorld {
    pub fn new(pair_cache: GridBroadphase, ball_obj: SphereRigidBody) -> Self {
        Self {
            collision_world: CollisionWorld::new(pair_cache, ball_obj),
            solver: SeqImpulseConstraintSolver::default(),
        }
    }

    #[inline]
    pub const fn ball_mut(&mut self) -> &mut SphereRigidBody {
        &mut self.collision_world.ball_obj
    }

    #[inline]
    pub const fn ball(&self) -> &SphereRigidBody {
        &self.collision_world.ball_obj
    }

    pub fn set_gravity(&mut self, gravity: Vec3A) {
        self.collision_world.ball_obj.set_gravity(gravity);
    }

    #[inline]
    pub fn add_rigid_body(&mut self, body: RigidBody) -> usize {
        self.collision_world.add_collision_obj(body)
    }

    #[inline]
    fn apply_gravity(&mut self) {
        self.collision_world.ball_obj.apply_gravity();
    }

    fn predict_unconstraint_motion(&mut self, time_step: f32) {
        let body = &mut self.collision_world.ball_obj;

        body.apply_damping(time_step);
        let predicted_trans = body.predict_integration_trans(time_step);
        body.interp_world_trans = predicted_trans;
    }

    #[inline]
    fn solve_constraints(&mut self, time_step: f32) {
        self.solver.solve_group(
            &mut self.collision_world.ball_obj,
            &mut self.collision_world.dispatcher1.manifolds,
            time_step,
        );
    }

    fn integrate_trans(&mut self, time_step: f32) {
        let body = &mut self.collision_world.ball_obj;
        let predicted_trans = body.predict_integration_trans(time_step);
        body.set_center_of_mass_trans(predicted_trans);
    }

    #[inline]
    const fn clear_forces(&mut self) {
        self.collision_world.ball_obj.clear_forces();
    }

    fn internal_single_step_simulation(
        &mut self,
        time_step: f32,
        contact_added_callback: &mut ArenaContactTracker,
    ) {
        self.predict_unconstraint_motion(time_step);

        self.collision_world
            .perform_discrete_collision_detection(contact_added_callback);

        self.solve_constraints(time_step);
        self.integrate_trans(time_step);
    }

    pub fn step_simulation(
        &mut self,
        time_step: f32,
        contact_added_callback: &mut ArenaContactTracker,
    ) {
        self.apply_gravity();
        self.internal_single_step_simulation(time_step, contact_added_callback);

        self.clear_forces();
    }
}
