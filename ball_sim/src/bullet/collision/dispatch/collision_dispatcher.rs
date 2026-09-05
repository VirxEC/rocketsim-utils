use super::{
    convex_concave_collision_alg::{self, PendingSphereContact},
    convex_plane_collision_alg,
};
use crate::{
    ArenaContactTracker,
    bullet::{
        collision::{
            broadphase::GridBroadphaseProxy,
            narrowphase::persistent_manifold::{PersistentManifold, pair_key},
            shapes::collision_shape::CollisionShapes,
        },
        dynamics::{rigid_body::RigidBody, sphere_rigid_body::SphereRigidBody},
    },
};

/// Index of the ball in persistent-manifold pair keys.
/// The ball is always `body0`; static bodies use `client_obj_idx + 1`
/// (the `+ 1` keeps every static distinct from the ball).
const BALL_BODY_IDX: usize = 0;

#[inline]
const fn static_body_idx(client_obj_idx: usize) -> usize {
    client_obj_idx + 1
}

#[derive(Clone, Debug)]
pub struct CollisionDispatcher {
    pub persistent_manifolds: Vec<PersistentManifold>,
    /// Indices into `persistent_manifolds` with contacts this tick.
    /// Pushed in pair-processing order, so the solver sees the same
    /// contact order as the old per-tick manifold clones.
    pub active_manifolds: Vec<usize>,
    /// Dense index from ordered body pair to `persistent_manifolds` index.
    /// Cell `min * stride + max` holds the index plus one (`0` = none).
    /// Push-only manifold vector, so indices stay stable. Maintain at push
    /// sites below; any removal/clear must rebuild this table too.
    manifold_table: Vec<u32>,
    manifold_stride: usize,
    sphere_contact_scratch: Vec<PendingSphereContact>,
}

impl Default for CollisionDispatcher {
    fn default() -> Self {
        Self {
            persistent_manifolds: Vec::with_capacity(8),
            active_manifolds: Vec::with_capacity(8),
            manifold_table: Vec::new(),
            manifold_stride: 0,
            // Ball-vs-mesh sweeps collect a few triangle hits;
            // keep the buffer across ticks instead of reallocating.
            sphere_contact_scratch: Vec::with_capacity(16),
        }
    }
}

impl CollisionDispatcher {
    /// Push a first-seen pair's manifold and record its index.
    /// Stamps the ball/static pair indices onto the manifold (ball_sim
    /// bodies carry no pair indices, unlike upstream's `world_array_idx`).
    fn insert_persistent_manifold(
        &mut self,
        body0_idx: usize,
        body1_idx: usize,
        key: u64,
        mut manifold: PersistentManifold,
    ) -> usize {
        manifold.body0_idx = body0_idx;
        manifold.body1_idx = body1_idx;
        manifold.pair_key = key;
        // Read indices before push; growth rebuild covers prior manifolds only.
        let (lo, hi) = (body0_idx.min(body1_idx), body0_idx.max(body1_idx));
        if hi >= self.manifold_stride {
            self.grow_manifold_table(hi + 1);
        }
        self.persistent_manifolds.push(manifold);
        let idx = self.persistent_manifolds.len() - 1;
        let table_idx = lo * self.manifold_stride + hi;
        debug_assert_eq!(self.manifold_table[table_idx], 0);
        self.manifold_table[table_idx] = u32::try_from(idx + 1).expect("manifold index overflow");
        idx
    }

    /// Grow the table to cover `needed - 1` and reinsert existing mappings.
    fn grow_manifold_table(&mut self, needed: usize) {
        // Round stride to 16 to avoid reallocating per body.
        let new_stride = (needed + 15) & !15;
        let mut new_table = vec![0u32; new_stride * new_stride];
        for (idx, manifold) in self.persistent_manifolds.iter().enumerate() {
            let lo = manifold.body0_idx.min(manifold.body1_idx);
            let hi = manifold.body0_idx.max(manifold.body1_idx);
            new_table[lo * new_stride + hi] =
                u32::try_from(idx + 1).expect("manifold index overflow");
        }
        self.manifold_table = new_table;
        self.manifold_stride = new_stride;
    }

    // Miss leaves None; hit writes Some.
    fn process_collision(
        col_obj_a: &SphereRigidBody,
        col_obj_b: &RigidBody,
        contact_added_callback: &mut ArenaContactTracker,
        out: &mut Option<PersistentManifold>,
    ) {
        debug_assert!(out.is_none());
        match col_obj_b.get_collision_shape() {
            CollisionShapes::StaticPlane(plane) => convex_plane_collision_alg::process_collision(
                col_obj_a,
                col_obj_b,
                plane,
                contact_added_callback,
                out,
            ),
            // Mesh collisions reuse a persistent manifold via
            // `process_collision_into`; see `near_callback`.
            CollisionShapes::TriangleMesh(_) => unreachable!(),
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

        // Dense table lookup; insertion order is unchanged.
        let (body0_idx, body1_idx) = (BALL_BODY_IDX, static_body_idx(proxy1.client_obj_idx));
        let wanted = pair_key(body0_idx, body1_idx);
        let (lo, hi) = (body0_idx.min(body1_idx), body0_idx.max(body1_idx));
        let cached_idx = if hi < self.manifold_stride {
            let cell = self.manifold_table[lo * self.manifold_stride + hi];
            if cell == 0 {
                None
            } else {
                Some(cell as usize - 1)
            }
        } else {
            None
        };
        if let Some(cached_idx) = cached_idx {
            // Push-only vector, so a hit must reference this exact pair.
            let manifold = &self.persistent_manifolds[cached_idx];
            debug_assert_eq!(manifold.pair_key, wanted);
            debug_assert!(
                manifold.body0_idx == body0_idx && manifold.body1_idx == body1_idx,
                "stale pair index"
            );
        }

        if let CollisionShapes::TriangleMesh(mesh) = rb1.get_collision_shape() {
            let persistent_idx = if let Some(cached_idx) = cached_idx {
                cached_idx
            } else {
                self.insert_persistent_manifold(
                    body0_idx,
                    body1_idx,
                    wanted,
                    // Upstream caches the threshold per body and takes the
                    // min for the pair. (The static mesh body's world-space
                    // disc dwarfs the ball's, so this equals the ball's.)
                    PersistentManifold::new(
                        rb0.get_contact_breaking_threshold()
                            .min(rb1.get_contact_breaking_threshold()),
                    ),
                )
            };
            let has_contacts = convex_concave_collision_alg::process_collision_into(
                rb0,
                rb1,
                mesh,
                &mut self.persistent_manifolds[persistent_idx],
                &mut self.sphere_contact_scratch,
                contact_added_callback,
            );

            if has_contacts {
                self.active_manifolds.push(persistent_idx);
            }
            return;
        }

        let mut fresh: Option<PersistentManifold> = None;
        Self::process_collision(rb0, rb1, contact_added_callback, &mut fresh);

        // Share the persistent manifold with the solver by index instead
        // of pushing a per-tick clone. Merge and refresh order mirrors
        // upstream: cull separated points before merging fresh
        // detections, and skip the no-op empty refresh.
        let active_idx = match (cached_idx, fresh) {
            (Some(cached_idx), Some(fresh_manifold)) => {
                // Skip the no-op empty refresh (see `refresh_contact_points`);
                // the merge below behaves identically on an empty cache either way.
                if !self.persistent_manifolds[cached_idx].point_cache.is_empty() {
                    self.persistent_manifolds[cached_idx].refresh_contact_points(rb0, rb1);
                }
                self.persistent_manifolds[cached_idx].merge_contact_points(&fresh_manifold);
                cached_idx
            }
            (Some(cached_idx), None) => {
                // Skip the no-op empty refresh; an empty manifold stays
                // empty and the active push below is already guarded.
                if !self.persistent_manifolds[cached_idx].point_cache.is_empty() {
                    self.persistent_manifolds[cached_idx].refresh_contact_points(rb0, rb1);
                }
                cached_idx
            }
            (None, Some(fresh_manifold)) => {
                self.insert_persistent_manifold(body0_idx, body1_idx, wanted, fresh_manifold)
            }
            (None, None) => return,
        };

        if !self.persistent_manifolds[active_idx].point_cache.is_empty() {
            self.active_manifolds.push(active_idx);
        }
    }
}
