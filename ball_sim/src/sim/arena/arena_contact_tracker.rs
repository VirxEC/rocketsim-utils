use crate::bullet::{
    collision::{
        dispatch::internal_edge_utility::adjust_internal_edge_contacts,
        narrowphase::manifold_point::ManifoldPoint,
    },
    dynamics::rigid_body::RigidBody,
};

// Store one contact event.
#[derive(Debug, Copy, Clone)]
pub struct ContactRecord {
    pub manifold_point: ManifoldPoint,
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
        // In ball_sim `body_b` is always a static world body (the ball is a
        // `SphereRigidBody` and never appears here); upstream additionally
        // requires `body_b.is_static_obj()`.
        manifold_point.is_special = true;

        // Record contact data before edge adjustment changes the manifold.
        if manifold_point.is_special {
            // Save the raw normal for special-contact aggregation.
            manifold_point.raw_normal_world_on_b = manifold_point.normal_world_on_b;
        }

        self.collision_records.push(ContactRecord {
            manifold_point: *manifold_point,
        });

        if let Some(idx) = triangle_idx {
            adjust_internal_edge_contacts(manifold_point, body_b, idx);
        }
    }
}
