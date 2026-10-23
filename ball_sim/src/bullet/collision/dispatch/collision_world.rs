use glam::Vec3A;

use super::collision_dispatcher::CollisionDispatcher;
use crate::{
    ArenaContactTracker,
    bullet::{
        collision::{
            broadphase::GridBroadphase,
            narrowphase::persistent_manifold::CONTACT_BREAKING_THRESHOLD,
        },
        dynamics::{rigid_body::RigidBody, sphere_rigid_body::SphereRigidBody},
    },
};

#[derive(Clone)]
pub struct CollisionWorld {
    pub ball_obj: SphereRigidBody,
    collision_objs: Vec<RigidBody>,
    pub dispatcher1: CollisionDispatcher,
    broadphase_pair_cache: GridBroadphase,
}

impl CollisionWorld {
    const CBT: Vec3A = Vec3A::splat(CONTACT_BREAKING_THRESHOLD);

    pub fn new(pair_cache: GridBroadphase, ball_obj: SphereRigidBody) -> Self {
        Self {
            ball_obj,
            collision_objs: Vec::new(),
            dispatcher1: CollisionDispatcher::default(),
            broadphase_pair_cache: pair_cache,
        }
    }

    pub fn init_ball_proxy(&mut self) {
        let aabb = self
            .ball_obj
            .get_collision_shape()
            .get_aabb(self.ball_obj.get_world_trans());
        self.broadphase_pair_cache
            .create_ball_proxy(aabb, &mut self.dispatcher1);
    }

    pub fn add_collision_obj(&mut self, mut obj: RigidBody) -> usize {
        let idx = self.collision_objs.len();
        let aabb = obj
            .get_collision_shape()
            .get_world_aabb(obj.get_world_trans());

        obj.broadphase_handle =
            self.broadphase_pair_cache
                .create_static_proxy(aabb, idx + 1, &mut self.dispatcher1);

        self.collision_objs.push(obj);
        idx
    }

    fn update_aabbs(&mut self) {
        let shape = self.ball_obj.get_collision_shape();
        let aabb = shape.get_aabb_covering(
            self.ball_obj.get_world_trans(),
            self.ball_obj.interp_world_trans,
        );

        let mut aabb = aabb;
        aabb.min -= Self::CBT;
        aabb.max += Self::CBT;

        debug_assert!((aabb.max - aabb.min).length_squared() < 1e12);
        self.broadphase_pair_cache
            .set_aabb(aabb, &mut self.dispatcher1);
    }

    pub fn perform_discrete_collision_detection(
        &mut self,
        contact_added_callback: &mut ArenaContactTracker,
    ) {
        self.dispatcher1.active_manifolds.clear();

        self.update_aabbs();

        self.broadphase_pair_cache.process_all_overlapping_pairs(
            &self.ball_obj,
            &self.collision_objs,
            &mut self.dispatcher1,
            contact_added_callback,
        );

        sort_active_manifolds(&mut self.dispatcher1.active_manifolds);
    }
}

/// Reproduce RocketSim's island ordering for a one-dynamic-body world.
///
/// Every active manifold is ball-vs-static, so every island key is the same
/// value. RocketSim still runs `indices.sort_unstable()` followed by its
/// equal-key Hoare pass (whose comparisons never move `i`/`j`), which is a
/// fixed permutation of the active list.
fn sort_active_manifolds(indices: &mut [usize]) {
    if indices.len() <= 1 {
        return;
    }

    indices.sort_unstable();

    fn sort(indices: &mut [usize], lo: usize, hi: usize) {
        let mut i = lo;
        let mut j = hi;
        loop {
            if i > j {
                break;
            }
            indices.swap(i, j);
            i += 1;
            if j == 0 {
                break;
            }
            j -= 1;
            if i > j {
                break;
            }
        }
        if lo < j {
            sort(indices, lo, j);
        }
        if i < hi {
            sort(indices, i, hi);
        }
    }

    sort(indices, 0, indices.len() - 1);
}
