use glam::Vec3A;

use super::rigid_body::{Impulse, RigidBody};

#[derive(Clone, Copy, Debug)]
pub struct DiscreteDynamicsWorld {
    pub collision_obj: RigidBody,
}

impl DiscreteDynamicsWorld {
    /// Applies gravity, then applies all accumulated impulses and integrates the body.
    ///
    /// Accumulated impulses must be cleared before accumulating new ones
    /// (see [`Self::clear_accum_forces`]).
    pub fn step_simulation(&mut self, gravity: Vec3A, tick_time: f32) {
        self.collision_obj
            .add_impulse(Impulse::Linear(gravity * tick_time), false, true);

        let rb = &mut self.collision_obj;
        rb.lin_vel += rb.accum_lin_vel;
        rb.ang_vel += rb.accum_ang_vel;
        rb.integrate_trans(tick_time);
    }

    pub fn clear_accum_forces(&mut self) {
        self.collision_obj.clear_accum_vels();
    }
}
