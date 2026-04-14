use crate::bullet::dynamics::rigid_body::RigidBody;

pub struct CollisionWorld {
    pub collision_objs: Vec<RigidBody>,
}

impl CollisionWorld {
    pub const fn new() -> Self {
        Self {
            collision_objs: Vec::new(),
        }
    }

    pub fn add_collision_obj(&mut self, mut obj: RigidBody) -> usize {
        obj.world_array_idx = self.collision_objs.len();

        let idx = self.collision_objs.len();
        self.collision_objs.push(obj);

        idx
    }
}
