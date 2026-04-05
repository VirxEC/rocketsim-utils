use crate::bullet::{
    collision::{
        dispatch::internal_edge_utility::adjust_internal_edge_contacts,
        narrowphase::manifold_point::ManifoldPoint,
    },
    dynamics::rigid_body::RigidBody,
};

// An instance of a contact event
#[derive(Debug, Copy, Clone)]
pub(crate) struct ContactRecord {
    pub manifold_point: ManifoldPoint,
}

// A struct to be accessed through the bullet contact callbacks
pub(crate) struct ArenaContactTracker {
    collision_records: Vec<ContactRecord>,
}

impl ArenaContactTracker {
    pub fn new() -> Self {
        Self {
            collision_records: Vec::with_capacity(4), // Rarely exceeded
        }
    }

    pub fn num_records(&self) -> usize {
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
        // NOTE: Push *before* the manifold is mutated by adjust_internal_edge_contacts()
        self.collision_records.push(ContactRecord {
            manifold_point: *manifold_point,
        });

        if let Some(idx) = triangle_idx {
            adjust_internal_edge_contacts(manifold_point, body_b, idx);
        }
    }
}
