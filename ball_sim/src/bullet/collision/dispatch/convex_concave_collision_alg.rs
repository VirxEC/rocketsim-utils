use glam::Vec3A;

use crate::{
    ArenaContactTracker,
    bullet::{
        collision::{
            narrowphase::persistent_manifold::PersistentManifold,
            shapes::{
                bvh_triangle_mesh_shape::BvhTriangleMeshShape, triangle_callback::ProcessTriangle,
                triangle_shape::TriangleShape,
            },
        },
        dynamics::{rigid_body::RigidBody, sphere_rigid_body::SphereRigidBody},
    },
    shared::Aabb,
};

#[derive(Clone, Copy, Debug)]
pub(crate) struct PendingSphereContact {
    normal_on_b: Vec3A,
    point_in_world: Vec3A,
    depth: f32,
    triangle_idx: usize,
}

struct ConvexTriangleCallback<'a> {
    pub collected: &'a mut Vec<PendingSphereContact>,
    pub tri_obj: &'a RigidBody,
    sphere_center: Vec3A,
    sphere_radius: f32,
    contact_breaking_threshold: f32,
}

impl<'a> ConvexTriangleCallback<'a> {
    pub fn new(
        collected: &'a mut Vec<PendingSphereContact>,
        tri_obj: &'a RigidBody,
        sphere_center: Vec3A,
        sphere_radius: f32,
        contact_breaking_threshold: f32,
    ) -> Self {
        Self {
            collected,
            tri_obj,
            sphere_center,
            sphere_radius,
            contact_breaking_threshold,
        }
    }
}

impl ProcessTriangle for ConvexTriangleCallback<'_> {
    fn process_triangle(
        &mut self,
        triangle: &TriangleShape,
        _tri_aabb: &Aabb,
        triangle_idx: usize,
    ) {
        let Some(contact_info) = triangle.intersect_sphere(
            self.sphere_center,
            self.sphere_radius,
            self.contact_breaking_threshold,
        ) else {
            return;
        };

        // Keep only front-side triangle contacts.
        let center_to_tri = self.sphere_center - triangle.points[0];
        if center_to_tri.dot(triangle.normal) < 0.0 {
            return;
        }

        // Triangles are mesh-local; the mesh body carries only a
        // translation (identity rotation), so world points are local
        // points shifted by the body origin.
        let tri_trans = self.tri_obj.get_world_trans();
        let normal_on_b = contact_info.result_normal;
        let point_in_world = contact_info.contact_point + tri_trans;

        self.collected.push(PendingSphereContact {
            normal_on_b,
            point_in_world,
            depth: contact_info.depth,
            triangle_idx,
        });
    }
}

pub fn process_collision_into(
    convex_obj: &SphereRigidBody,
    concave_obj: &RigidBody,
    tri_mesh: &BvhTriangleMeshShape,
    manifold: &mut PersistentManifold,
    scratch: &mut Vec<PendingSphereContact>,
    contact_added_callback: &mut ArenaContactTracker,
) -> bool {
    manifold.most_recently_evicted_point = None;
    scratch.clear();

    // Mesh BVHs hold local triangles; the mesh body carries only a
    // translation, so the ball center in mesh-local space is the world
    // center minus the body origin.
    let xform1 = convex_obj.get_world_trans();
    let xform2 = concave_obj.get_world_trans();
    let convex_in_triangle_space = xform1 - xform2;

    let sphere_shape = convex_obj.get_collision_shape();
    let contact_breaking_threshold = manifold.get_contact_breaking_threshold();
    {
        let mut convex_triangle_callback = ConvexTriangleCallback::new(
            scratch,
            concave_obj,
            convex_in_triangle_space,
            sphere_shape.get_radius(),
            contact_breaking_threshold,
        );

        let aabb = sphere_shape.get_aabb(convex_in_triangle_space);
        tri_mesh.process_all_triangles(&mut convex_triangle_callback, &aabb);
    }

    for contact in scratch.iter() {
        manifold.add_contact_point(
            convex_obj,
            concave_obj,
            contact.normal_on_b,
            contact.point_in_world,
            contact.depth,
            Some(contact.triangle_idx),
            contact_added_callback,
        );
    }

    // Skip the no-op empty refresh (see `refresh_contact_points`).
    if !manifold.point_cache.is_empty() {
        manifold.refresh_contact_points(convex_obj, concave_obj);
    }

    !manifold.point_cache.is_empty()
}
