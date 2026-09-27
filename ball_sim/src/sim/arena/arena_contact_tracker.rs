use glam::Vec3A;

use crate::bullet::{
    collision::{
        dispatch::internal_edge_utility::adjust_internal_edge_contacts,
        narrowphase::manifold_point::ManifoldPoint,
    },
    dynamics::rigid_body::RigidBody,
};

#[derive(Debug, Copy, Clone)]
pub struct ContactRecord {
    pub pos_world_on_b: Vec3A,
    pub normal_world_on_b: Vec3A,
}

#[derive(Clone, Debug)]
// Track contacts reported by Bullet callbacks.
pub struct ArenaContactTracker {
    collision_records: Vec<ContactRecord>,
}

impl Default for ArenaContactTracker {
    #[inline]
    fn default() -> Self {
        Self {
            collision_records: Vec::with_capacity(4), // Reserve space for common contact counts.
        }
    }
}

impl ArenaContactTracker {
    pub const fn num_records(&self) -> usize {
        self.collision_records.len()
    }

    pub fn get_record(&self, idx: usize) -> &ContactRecord {
        &self.collision_records[idx]
    }

    pub fn clear_records(&mut self) {
        self.collision_records.clear();
    }
}

impl ArenaContactTracker {
    pub fn callback(
        &mut self,
        manifold_point: &mut ManifoldPoint,
        body_b: &RigidBody,
        triangle_idx: Option<usize>,
    ) {
        manifold_point.is_special = true;

        // Record contact data before edge adjustment changes the manifold.
        self.collision_records.push(ContactRecord {
            pos_world_on_b: manifold_point.pos_world_on_b,
            normal_world_on_b: manifold_point.normal_world_on_b,
        });

        if let Some(idx) = triangle_idx {
            adjust_internal_edge_contacts(manifold_point, body_b, idx);
        }
    }
}
