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

struct ConvexTriangleCallback<'a> {
    pub manifold: &'a mut PersistentManifold,
    pub convex_obj: &'a SphereRigidBody,
    pub tri_obj: &'a RigidBody,
    sphere_center: Vec3A,
    sphere_radius: f32,
    radius_with_threshold: f32,
    radius_with_threshold_sqr: f32,
    contact_added_callback: &'a mut ArenaContactTracker,
}

impl<'a> ConvexTriangleCallback<'a> {
    pub fn new(
        manifold: &'a mut PersistentManifold,
        convex_obj: &'a SphereRigidBody,
        tri_obj: &'a RigidBody,
        sphere_center: Vec3A,
        sphere_radius: f32,
        contact_breaking_threshold: f32,
        contact_added_callback: &'a mut ArenaContactTracker,
    ) -> Self {
        let radius_with_threshold = sphere_radius + contact_breaking_threshold;
        let radius_with_threshold_sqr = radius_with_threshold * radius_with_threshold;

        Self {
            manifold,
            convex_obj,
            tri_obj,
            sphere_center,
            sphere_radius,
            radius_with_threshold,
            radius_with_threshold_sqr,
            contact_added_callback,
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
        let Some(contact_info) = triangle.intersect_sphere_front_precomputed(
            self.sphere_center,
            self.sphere_radius,
            self.radius_with_threshold,
            self.radius_with_threshold_sqr,
        ) else {
            return;
        };

        let tri_trans = self.tri_obj.get_world_trans();
        let normal_on_b = contact_info.result_normal;
        let point_in_world = contact_info.contact_point + tri_trans;

        self.manifold.add_contact_point(
            self.convex_obj,
            self.tri_obj,
            normal_on_b,
            point_in_world,
            contact_info.depth,
            Some(triangle_idx),
            self.contact_added_callback,
        );
    }
}

pub fn process_collision_into(
    convex_obj: &SphereRigidBody,
    concave_obj: &RigidBody,
    tri_mesh: &BvhTriangleMeshShape,
    manifold: &mut PersistentManifold,
    contact_added_callback: &mut ArenaContactTracker,
) -> bool {
    let xform1 = convex_obj.get_world_trans();
    let xform2 = concave_obj.get_world_trans();
    let convex_in_triangle_space = xform1 - xform2;

    let sphere_shape = convex_obj.get_collision_shape();
    let contact_breaking_threshold = manifold.get_contact_breaking_threshold();
    {
        let mut convex_triangle_callback = ConvexTriangleCallback::new(
            manifold,
            convex_obj,
            concave_obj,
            convex_in_triangle_space,
            sphere_shape.get_radius(),
            contact_breaking_threshold,
            contact_added_callback,
        );

        let aabb = sphere_shape.get_aabb(convex_in_triangle_space);
        tri_mesh.process_all_triangles(&mut convex_triangle_callback, &aabb);
    }

    // Skip the no-op empty refresh (see `refresh_contact_points`).
    if !manifold.point_cache.is_empty() {
        manifold.refresh_contact_points(convex_obj, concave_obj);
    }

    !manifold.point_cache.is_empty()
}
