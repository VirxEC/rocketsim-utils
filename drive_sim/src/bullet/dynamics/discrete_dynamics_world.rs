use glam::Vec3A;

use super::rigid_body::RigidBody;

#[derive(Clone, Copy, Debug)]
pub struct DiscreteDynamicsWorld {
    pub collision_obj: RigidBody,
}

impl DiscreteDynamicsWorld {
    pub fn step_simulation(&mut self, gravity: Vec3A, tick_time: f32) {
        self.collision_obj.accum_lin_vel += gravity * tick_time;

        let rb = &mut self.collision_obj;
        rb.lin_vel += rb.accum_lin_vel;
        rb.ang_vel += rb.accum_ang_vel;
        rb.integrate_trans(tick_time);
    }

    pub fn clear_accum_forces(&mut self) {
        self.collision_obj.clear_accum_vels();
    }
}
