use super::{
    axis_sweep::AxisSweep3,
    broadphase_proxy::{BroadphaseProxy, CollisionFilterGroups},
};
use crate::{
    ArenaContactTracker,
    bullet::{
        collision::dispatch::collision_dispatcher::CollisionDispatcher,
        dynamics::{rigid_body::RigidBody, sphere_rigid_body::SphereRigidBody},
    },
    shared::Aabb,
};

/// Pair broadphase for the ball-only world.
///
/// The ball is the only dynamic handle; all other handles are static arena
/// components. Pairs and their retained order come from the axis sweep.
#[derive(Clone)]
pub struct GridBroadphase {
    axis_sweep: AxisSweep3,
    ball_handle: usize,
}

impl GridBroadphase {
    pub fn new(_min_pos: glam::Vec3A, _max_pos: glam::Vec3A, _cell_size: f32) -> Self {
        Self {
            axis_sweep: AxisSweep3::new(),
            ball_handle: 0,
        }
    }

    pub fn create_ball_proxy(&mut self, aabb: Aabb, dispatcher: &mut CollisionDispatcher) -> usize {
        let proxy = BroadphaseProxy {
            client_obj_idx: 0,
            collision_filter_group: CollisionFilterGroups::Default as u8
                | CollisionFilterGroups::HoopsNet as u8
                | CollisionFilterGroups::DropshotTile as u8,
            collision_filter_mask: CollisionFilterGroups::ALL,
            unique_id: 0,
        };
        self.ball_handle = self.axis_sweep.add_handle(aabb, proxy, dispatcher);
        self.ball_handle
    }

    pub fn create_static_proxy(
        &mut self,
        aabb: Aabb,
        client_obj_idx: usize,
        dispatcher: &mut CollisionDispatcher,
    ) -> usize {
        let proxy = BroadphaseProxy {
            client_obj_idx,
            collision_filter_group: CollisionFilterGroups::Static as u8,
            collision_filter_mask: CollisionFilterGroups::ALL ^ CollisionFilterGroups::Static,
            unique_id: 0,
        };
        self.axis_sweep.add_handle(aabb, proxy, dispatcher)
    }

    pub fn set_aabb(&mut self, aabb: Aabb, dispatcher: &mut CollisionDispatcher) {
        self.axis_sweep.set_aabb(self.ball_handle, aabb, dispatcher);
    }

    pub fn process_all_overlapping_pairs(
        &mut self,
        ball_obj: &SphereRigidBody,
        collision_objs: &[RigidBody],
        dispatcher: &mut CollisionDispatcher,
        contact_added_callback: &mut ArenaContactTracker,
    ) {
        let handles = self.axis_sweep.handles();
        // The ball handle is allocated after every static handle, so the
        // pair cache always stores it as `proxy1` (ordered by unique ID).
        for pair in self.axis_sweep.retained_pairs() {
            debug_assert_eq!(pair.proxy1, self.ball_handle);
            let other = &handles[pair.proxy0];
            dispatcher.near_callback(ball_obj, collision_objs, other, contact_added_callback);
        }
    }
}
