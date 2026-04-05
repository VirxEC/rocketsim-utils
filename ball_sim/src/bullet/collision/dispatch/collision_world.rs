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

pub struct CollisionWorld {
    pub ball_obj: SphereRigidBody,
    collision_objs: Vec<RigidBody>,
    pub dispatcher1: CollisionDispatcher,
    broadphase_pair_cache: GridBroadphase,
}

impl CollisionWorld {
    const CBT: Vec3A = Vec3A::splat(CONTACT_BREAKING_THRESHOLD);

    pub fn new(
        dispatcher: CollisionDispatcher,
        mut pair_cache: GridBroadphase,
        ball_obj: SphereRigidBody,
    ) -> Self {
        let aabb = ball_obj
            .get_collision_shape()
            .get_aabb(ball_obj.get_world_trans());

        pair_cache.create_proxy(aabb);

        Self {
            ball_obj,
            collision_objs: Vec::new(),
            dispatcher1: dispatcher,
            broadphase_pair_cache: pair_cache,
        }
    }

    pub fn add_collision_obj(&mut self, mut obj: RigidBody) -> usize {
        let idx = self.collision_objs.len();
        let mut aabb = obj.get_collision_shape().get_aabb();

        aabb.min -= Self::CBT;
        aabb.max += Self::CBT;

        obj.broadphase_handle = self
            .broadphase_pair_cache
            .create_static_proxy(aabb, &obj, idx);

        self.collision_objs.push(obj);
        idx
    }

    fn update_aabbs(&mut self) {
        let mut aabb = self
            .ball_obj
            .get_collision_shape()
            .get_aabb(self.ball_obj.get_world_trans());

        aabb.min -= Self::CBT;
        aabb.max += Self::CBT;

        let mut aabb2 = self
            .ball_obj
            .get_collision_shape()
            .get_aabb(self.ball_obj.interp_world_trans);
        aabb2.min -= Self::CBT;
        aabb2.max += Self::CBT;
        aabb += aabb2;

        debug_assert!((aabb.max - aabb.min).length_squared() < 1e12);
        self.broadphase_pair_cache.set_aabb(aabb);
    }

    pub fn perform_discrete_collision_detection(
        &mut self,
        contact_added_callback: &mut ArenaContactTracker,
    ) {
        self.update_aabbs();

        self.broadphase_pair_cache.calculate_overlapping_pairs();
        self.broadphase_pair_cache.process_all_overlapping_pairs(
            &self.ball_obj,
            &self.collision_objs,
            &mut self.dispatcher1,
            contact_added_callback,
        );
    }
}
