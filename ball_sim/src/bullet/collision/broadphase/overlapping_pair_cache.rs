use arrayvec::ArrayVec;

use super::grid_broadphase::GridBroadphaseProxy;
use crate::{
    ArenaContactTracker,
    bullet::{
        collision::dispatch::collision_dispatcher::CollisionDispatcher,
        dynamics::{rigid_body::RigidBody, sphere_rigid_body::SphereRigidBody},
    },
};

pub struct HashedOverlappingPairCache {
    overlapping_pair_array: ArrayVec<usize, 4>,
}

impl Default for HashedOverlappingPairCache {
    fn default() -> Self {
        Self {
            overlapping_pair_array: ArrayVec::new(),
        }
    }
}

impl HashedOverlappingPairCache {
    pub fn add_overlapping_pair(&mut self, proxy1_idx: usize) {
        self.overlapping_pair_array.push(proxy1_idx);
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.overlapping_pair_array.is_empty()
    }

    pub fn process_all_overlapping_pairs(
        &mut self,
        ball_obj: &SphereRigidBody,
        collision_objs: &[RigidBody],
        dispatcher: &mut CollisionDispatcher,
        handles: &[GridBroadphaseProxy],
        contact_added_callback: &mut ArenaContactTracker,
    ) {
        for &proxy1_idx in &self.overlapping_pair_array {
            dispatcher.near_callback(
                ball_obj,
                collision_objs,
                &handles[proxy1_idx],
                contact_added_callback,
            );
        }

        self.overlapping_pair_array.clear();
    }
}
