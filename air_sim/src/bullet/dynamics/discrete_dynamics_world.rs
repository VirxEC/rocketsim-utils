use glam::Vec3A;

use super::{
    constraint_solver::seq_impulse_constraint_solver::SeqImpulseConstraintSolver,
    rigid_body::{RigidBody, RigidBodyFlags},
};
use crate::bullet::collision::collision_world::CollisionWorld;

pub struct DiscreteDynamicsWorld {
    collision_world: CollisionWorld,
    solver: SeqImpulseConstraintSolver,
    gravity: Vec3A,
}

impl DiscreteDynamicsWorld {
    pub fn new(constraint_solver: SeqImpulseConstraintSolver) -> Self {
        Self {
            collision_world: CollisionWorld::new(),
            solver: constraint_solver,
            gravity: Vec3A::new(0.0, -10.0, 0.0),
        }
    }

    #[inline]
    pub fn bodies_mut(&mut self) -> &mut [RigidBody] {
        &mut self.collision_world.collision_objs
    }

    pub const fn set_gravity(&mut self, gravity: Vec3A) {
        self.gravity = gravity;
    }

    #[inline]
    fn add_collision_obj(&mut self, body: RigidBody) -> usize {
        self.collision_world.add_collision_obj(body)
    }

    pub fn add_rigid_body(&mut self, mut body: RigidBody) -> usize {
        if body.get_rb_flags() & RigidBodyFlags::DisableWorldGravity as u8 == 0 {
            body.set_gravity(self.gravity);
        }

        self.add_collision_obj(body)
    }

    fn apply_gravity(&mut self) {
        for body in &mut self.collision_world.collision_objs {
            body.apply_gravity();
        }
    }

    fn predict_unconstraint_motion(&mut self, time_step: f32) {
        for body in &mut self.collision_world.collision_objs {
            body.apply_damping(time_step);
            let predicted_trans = body.predict_integration_trans(time_step);
            body.interp_world_trans = predicted_trans;
        }
    }

    #[inline]
    fn solve_constraints(&mut self, time_step: f32) {
        self.solver
            .solve_group(&mut self.collision_world.collision_objs, time_step);
    }

    fn integrate_transs(&mut self, time_step: f32) {
        for body in &mut self.collision_world.collision_objs {
            let predicted_trans = body.predict_integration_trans(time_step);
            body.set_center_of_mass_trans(predicted_trans);
        }
    }

    fn clear_forces(&mut self) {
        for body in &mut self.collision_world.collision_objs {
            body.clear_forces();
        }
    }

    fn internal_single_step_simulation(&mut self, time_step: f32) {
        self.predict_unconstraint_motion(time_step);
        self.solve_constraints(time_step);
        self.integrate_transs(time_step);
    }

    pub fn step_simulation(&mut self, time_step: f32) {
        self.apply_gravity();
        self.internal_single_step_simulation(time_step);

        self.clear_forces();
    }
}
