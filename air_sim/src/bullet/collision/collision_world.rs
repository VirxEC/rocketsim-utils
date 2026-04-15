use crate::bullet::dynamics::rigid_body::RigidBody;

pub struct CollisionWorld {
    pub collision_obj: RigidBody,
}

impl CollisionWorld {
    pub const fn new(obj: RigidBody) -> Self {
        Self { collision_obj: obj }
    }
}
