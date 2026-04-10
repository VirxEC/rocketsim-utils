use arrayvec::ArrayVec;

use super::{convex_concave_collision_alg, convex_plane_collision_alg};
use crate::{
    ArenaContactTracker,
    bullet::{
        collision::{
            broadphase::GridBroadphaseProxy, narrowphase::persistent_manifold::PersistentManifold,
            shapes::collision_shape::CollisionShapes,
        },
        dynamics::{rigid_body::RigidBody, sphere_rigid_body::SphereRigidBody},
    },
};

#[derive(Clone, Debug)]
pub struct CollisionDispatcher {
    pub manifolds: ArrayVec<PersistentManifold, 4>,
}

impl Default for CollisionDispatcher {
    fn default() -> Self {
        Self {
            manifolds: ArrayVec::new(),
        }
    }
}

impl CollisionDispatcher {
    fn process_collision(
        col_obj_a: &SphereRigidBody,
        col_obj_b: &RigidBody,
        contact_added_callback: &mut ArenaContactTracker,
    ) -> Option<PersistentManifold> {
        match col_obj_b.get_collision_shape() {
            CollisionShapes::StaticPlane(plane) => convex_plane_collision_alg::process_collision(
                col_obj_a,
                col_obj_b,
                plane,
                contact_added_callback,
            ),
            CollisionShapes::TriangleMesh(mesh) => convex_concave_collision_alg::process_collision(
                col_obj_a,
                col_obj_b,
                mesh,
                contact_added_callback,
            ),
        }
    }

    pub fn near_callback(
        &mut self,
        ball_obj: &SphereRigidBody,
        collision_objs: &[RigidBody],
        proxy1: &GridBroadphaseProxy,
        contact_added_callback: &mut ArenaContactTracker,
    ) {
        let rb0 = ball_obj;
        let rb1 = &collision_objs[proxy1.client_obj_idx];

        if let Some(manifold) = Self::process_collision(rb0, rb1, contact_added_callback) {
            self.manifolds.push(manifold);
        }
    }
}
