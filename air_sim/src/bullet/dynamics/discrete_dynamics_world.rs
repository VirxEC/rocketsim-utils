use glam::Vec3A;

use super::{
    constraint_solver::seq_impulse_constraint_solver::SeqImpulseConstraintSolver,
    rigid_body::RigidBody,
};
use crate::bullet::collision::collision_world::CollisionWorld;

pub struct DiscreteDynamicsWorld {
    collision_world: CollisionWorld,
    solver: SeqImpulseConstraintSolver,
}

impl DiscreteDynamicsWorld {
    pub fn new(constraint_solver: SeqImpulseConstraintSolver, obj: RigidBody) -> Self {
        Self {
            collision_world: CollisionWorld::new(obj),
            solver: constraint_solver,
        }
    }

    #[inline]
    pub const fn body_mut(&mut self) -> &mut RigidBody {
        &mut self.collision_world.collision_obj
    }

    pub fn set_gravity(&mut self, gravity: Vec3A) {
        self.collision_world.collision_obj.set_gravity(gravity);
    }

    #[inline]
    fn solve_constraints(&mut self, time_step: f32) {
        self.solver
            .solve_group(&mut self.collision_world.collision_obj, time_step);
    }

    pub fn step_simulation(&mut self, time_step: f32) {
        self.collision_world.collision_obj.apply_gravity();

        self.solve_constraints(time_step);
        self.collision_world
            .collision_obj
            .integration_trans(time_step);

        self.collision_world.collision_obj.clear_forces();
    }
}
