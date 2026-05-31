use glam::Vec3A;

use super::rigid_body::RigidBody;

#[derive(Clone, Copy, Debug)]
pub struct DiscreteDynamicsWorld {
    pub collision_obj: RigidBody,
}

impl DiscreteDynamicsWorld {
    pub fn step_simulation(&mut self, gravity: Vec3A, tick_time: f32) {
        self.collision_obj.apply_central_force(gravity);
        self.collision_obj.lin_vel += self.collision_obj.total_force * tick_time;
        self.collision_obj.integrate_trans(tick_time);
        self.collision_obj.clear_forces();
    }
}
