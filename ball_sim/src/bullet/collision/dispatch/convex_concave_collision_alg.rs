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
    pub manifold: PersistentManifold,
    pub convex_obj: &'a SphereRigidBody,
    pub tri_obj: &'a RigidBody,
    contact_added_callback: &'a mut ArenaContactTracker,
    sphere_center: Vec3A,
    sphere_radius: f32,
}

impl<'a> ConvexTriangleCallback<'a> {
    pub fn new(
        convex_obj: &'a SphereRigidBody,
        tri_obj: &'a RigidBody,
        sphere_center: Vec3A,
        sphere_radius: f32,
        contact_added_callback: &'a mut ArenaContactTracker,
    ) -> Self {
        Self {
            manifold: PersistentManifold::new(
                convex_obj
                    .get_collision_shape()
                    .get_contact_breaking_threshold(),
            ),
            convex_obj,
            tri_obj,
            sphere_center,
            sphere_radius,
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
        let Some(contact_info) = triangle.intersect_sphere(
            self.sphere_center,
            self.sphere_radius,
            self.manifold.contact_breaking_threshold,
        ) else {
            return;
        };

        let normal_on_b = contact_info.result_normal;
        let point_in_world = contact_info.contact_point;

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

pub fn process_collision(
    convex_obj: &SphereRigidBody,
    concave_obj: &RigidBody,
    tri_mesh: &BvhTriangleMeshShape,
    contact_added_callback: &mut ArenaContactTracker,
) -> Option<PersistentManifold> {
    let xform1 = convex_obj.get_world_trans();
    let xform2 = concave_obj.get_world_trans();
    let convex_in_triangle_space = xform1 - xform2;

    let sphere_shape = convex_obj.get_collision_shape();
    let mut convex_triangle_callback = ConvexTriangleCallback::new(
        convex_obj,
        concave_obj,
        convex_in_triangle_space,
        sphere_shape.get_radius(),
        contact_added_callback,
    );

    let aabb = sphere_shape.get_aabb(convex_in_triangle_space);
    tri_mesh.process_all_triangles(&mut convex_triangle_callback, &aabb);

    if convex_triangle_callback.manifold.point_cache.is_empty() {
        None
    } else {
        convex_triangle_callback
            .manifold
            .refresh_contact_points(convex_obj, concave_obj);
        Some(convex_triangle_callback.manifold)
    }
}
