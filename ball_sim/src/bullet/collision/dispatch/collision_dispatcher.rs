use super::{convex_concave_collision_alg, convex_plane_collision_alg};
use crate::{
    ArenaContactTracker,
    bullet::{
        collision::{
            broadphase::BroadphaseProxy,
            narrowphase::persistent_manifold::{PersistentManifold, pair_key},
            shapes::collision_shape::CollisionShapes,
        },
        dynamics::{rigid_body::RigidBody, sphere_rigid_body::SphereRigidBody},
    },
};

const BALL_BODY_IDX: usize = 0;

#[derive(Clone, Debug)]
pub struct CollisionDispatcher {
    pub persistent_manifolds: Vec<PersistentManifold>,
    /// Indices into `persistent_manifolds` with contacts this tick.
    /// Pushed in pair-processing order, so the solver sees the same
    /// contact order as the old per-tick manifold clones.
    pub active_manifolds: Vec<usize>,
    /// Dense index from the static body index to `persistent_manifolds`
    /// index. The ball is always body 0, so a pair is identified by its
    /// static body alone. Holds the manifold index plus one (`0` = none).
    manifold_table: Vec<u32>,
}

impl Default for CollisionDispatcher {
    fn default() -> Self {
        Self {
            persistent_manifolds: Vec::with_capacity(8),
            active_manifolds: Vec::with_capacity(8),
            manifold_table: Vec::new(),
        }
    }
}

impl CollisionDispatcher {
    /// Clear cached contacts after a teleport.
    ///
    /// Reused prediction arenas teleport to varied poses. Drop stale
    /// contacts. Match a fresh arena.
    pub fn clear_persistent_manifolds(&mut self) {
        self.persistent_manifolds.clear();
        self.active_manifolds.clear();
        self.manifold_table.clear();
    }

    /// Push a first-seen pair's manifold and record its index.
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
        let slot = body0_idx.max(body1_idx);
        if slot >= self.manifold_table.len() {
            self.grow_manifold_table(slot + 1);
        }
        self.persistent_manifolds.push(manifold);
        let idx = self.persistent_manifolds.len() - 1;
        debug_assert_eq!(self.manifold_table[slot], 0);
        self.manifold_table[slot] = u32::try_from(idx + 1).expect("manifold index overflow");
        idx
    }

    pub fn release_manifold(&mut self, body0_idx: usize, body1_idx: usize) {
        let slot = body0_idx.max(body1_idx);
        let Some(entry) = self.manifold_table.get(slot).copied() else {
            return;
        };
        if entry == 0 {
            return;
        }
        let index = entry as usize - 1;
        let last_index = self.persistent_manifolds.len() - 1;

        self.persistent_manifolds[index].point_cache.clear();
        self.manifold_table[slot] = 0;

        // A child-algorithm cleanup can release a manifold in the middle of
        // pair dispatch. Keep earlier active rows and remap the swap target.
        self.active_manifolds.retain(|&active| active != index);
        self.persistent_manifolds.swap_remove(index);
        if index != last_index {
            for active in &mut self.active_manifolds {
                if *active == last_index {
                    *active = index;
                }
            }
            let moved = &self.persistent_manifolds[index];
            let moved_slot = moved.body0_idx.max(moved.body1_idx);
            self.manifold_table[moved_slot] =
                u32::try_from(index + 1).expect("manifold index overflow");
        }
    }

    /// Grow the table to cover `needed - 1` and reinsert existing mappings.
    fn grow_manifold_table(&mut self, needed: usize) {
        let mut new_table = vec![0u32; needed];
        for (idx, manifold) in self.persistent_manifolds.iter().enumerate() {
            let slot = manifold.body0_idx.max(manifold.body1_idx);
            new_table[slot] = u32::try_from(idx + 1).expect("manifold index overflow");
        }
        self.manifold_table = new_table;
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
            CollisionShapes::TriangleMesh(_) => unreachable!(),
        }
    }

    pub fn near_callback(
        &mut self,
        ball_obj: &SphereRigidBody,
        collision_objs: &[RigidBody],
        proxy1: &BroadphaseProxy,
        contact_added_callback: &mut ArenaContactTracker,
    ) {
        let rb0 = ball_obj;
        let body1_idx = proxy1.client_obj_idx;
        let rb1 = &collision_objs[body1_idx - 1];

        // Dense table lookup; insertion order is unchanged.
        let body0_idx = BALL_BODY_IDX;
        let wanted = pair_key(body0_idx, body1_idx);
        let cached_idx = self
            .manifold_table
            .get(body1_idx)
            .copied()
            .filter(|&cell| cell != 0)
            .map(|cell| cell as usize - 1);
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
                    PersistentManifold::new(
                        rb0.get_contact_breaking_threshold()
                            .min(rb1.get_contact_breaking_threshold()),
                    ),
                )
            };
            convex_concave_collision_alg::process_collision_into(
                rb0,
                rb1,
                mesh,
                &mut self.persistent_manifolds[persistent_idx],
                contact_added_callback,
            );

            // RocketSim keeps the active row even when this tick produced no
            // points; the island sort below must see the same list length.
            self.active_manifolds.push(persistent_idx);
            return;
        }

        let mut fresh: Option<PersistentManifold> = None;
        Self::process_collision(rb0, rb1, contact_added_callback, &mut fresh);

        // Share the persistent manifold with the solver by index instead
        let active_idx = match (cached_idx, fresh) {
            (Some(cached_idx), Some(fresh_manifold)) => {
                if !self.persistent_manifolds[cached_idx].point_cache.is_empty() {
                    self.persistent_manifolds[cached_idx].refresh_contact_points(rb0, rb1);
                }
                self.persistent_manifolds[cached_idx].merge_contact_points(&fresh_manifold);
                cached_idx
            }
            (Some(cached_idx), None) => {
                if !self.persistent_manifolds[cached_idx].point_cache.is_empty() {
                    self.persistent_manifolds[cached_idx].refresh_contact_points(rb0, rb1);
                }
                cached_idx
            }
            (None, Some(fresh_manifold)) => {
                self.insert_persistent_manifold(body0_idx, body1_idx, wanted, fresh_manifold)
            }
            (None, None) => {
                // RocketSim retains an empty row after a pair survives the
                // broadphase gate, even before the first contact point.
                self.insert_persistent_manifold(
                    body0_idx,
                    body1_idx,
                    wanted,
                    PersistentManifold::new(
                        rb0.get_contact_breaking_threshold()
                            .min(rb1.get_contact_breaking_threshold()),
                    ),
                )
            }
        };

        // The island sort consumes every processed pair, including rows whose
        // point cache is empty this tick.
        self.active_manifolds.push(active_idx);
    }
}
