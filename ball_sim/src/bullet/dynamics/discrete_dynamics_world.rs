use glam::Vec3A;

use super::{
    constraint_solver::seq_impulse_constraint_solver::SeqImpulseConstraintSolver,
    rigid_body::RigidBody, sphere_rigid_body::SphereRigidBody,
};
use crate::{
    ArenaContactTracker,
    bullet::collision::{broadphase::GridBroadphase, dispatch::collision_world::CollisionWorld},
};

#[derive(Clone)]
pub struct DiscreteDynamicsWorld {
    collision_world: CollisionWorld,
    solver: SeqImpulseConstraintSolver,
    gravity: Vec3A,
}

impl DiscreteDynamicsWorld {
    pub fn new(pair_cache: GridBroadphase, ball_obj: SphereRigidBody) -> Self {
        Self {
            collision_world: CollisionWorld::new(pair_cache, ball_obj),
            solver: SeqImpulseConstraintSolver::default(),
            gravity: Vec3A::ZERO,
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
        self.gravity = gravity;
    }

    #[inline]
    pub fn add_rigid_body(&mut self, body: RigidBody) -> usize {
        self.collision_world.add_collision_obj(body)
    }

    pub fn init_ball_proxy(&mut self) {
        self.collision_world.init_ball_proxy();
    }

    #[inline]
    fn apply_gravity(&mut self, time_step: f32) {
        self.collision_world.ball_obj.accum_lin_vel += self.gravity * time_step;
    }

    fn predict_unconstraint_motion(&mut self, time_step: f32) {
        let body = &mut self.collision_world.ball_obj;

        body.apply_damping(time_step);
        let predicted_trans = body.predict_integration_trans(time_step);
        body.interp_world_trans = predicted_trans;
    }

    #[inline]
    fn solve_constraints(&mut self, time_step: f32) {
        let dispatcher = &mut self.collision_world.dispatcher1;
        let has_contact_points = dispatcher
            .active_manifolds
            .iter()
            .any(|&idx| !dispatcher.persistent_manifolds[idx].point_cache.is_empty());
        if !has_contact_points {
            // No contact rows. The solver copy-out would only add the
            // accumulated forces back into the ball, so do that directly.
            // Broadphase pairs without points still need their active rows
            // cleared, as the solver would do during setup.
            dispatcher.active_manifolds.clear();
            let body = &mut self.collision_world.ball_obj;
            body.lin_vel += body.accum_lin_vel;
            body.ang_vel += body.accum_ang_vel;
            return;
        }
        self.solver.solve_group(
            &mut self.collision_world.ball_obj,
            &mut dispatcher.persistent_manifolds,
            &mut dispatcher.active_manifolds,
            time_step,
        );
    }

    fn integrate_trans(&mut self, time_step: f32) {
        let body = &mut self.collision_world.ball_obj;
        let predicted_trans = body.predict_integration_trans(time_step);
        body.set_center_of_mass_trans(predicted_trans);
    }

    #[inline]
    pub const fn clear_accum_forces(&mut self) {
        self.collision_world.ball_obj.clear_accum_vels();
    }

    #[inline]
    pub fn clear_persistent_manifolds(&mut self) {
        self.collision_world
            .dispatcher1
            .clear_persistent_manifolds();
    }

    /// Count cached contact manifolds for reuse diagnostics.
    #[inline]
    #[must_use]
    pub fn num_persistent_manifolds(&self) -> usize {
        self.collision_world.dispatcher1.persistent_manifolds.len()
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
        self.apply_gravity(time_step);
        self.internal_single_step_simulation(time_step, contact_added_callback);
    }
}
