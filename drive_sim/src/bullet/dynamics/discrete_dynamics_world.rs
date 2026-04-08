use glam::Vec3A;

use super::rigid_body::RigidBody;
use crate::consts;

const GRAVITY: Vec3A = Vec3A::new(0.0, 0.0, consts::GRAVITY_Z * consts::UU_TO_BT);

pub struct DiscreteDynamicsWorld {
    pub collision_obj: RigidBody,
}

impl DiscreteDynamicsWorld {
    pub fn step_simulation(&mut self, tick_time: f32) {
        self.collision_obj.apply_central_force(GRAVITY);
        self.collision_obj.lin_vel += self.collision_obj.total_force * tick_time;
        self.collision_obj.integrate_trans(tick_time);
        self.collision_obj.clear_forces();
    }
}
