use crate::{
    ArenaContactTracker,
    bullet::{
        collision::{
            narrowphase::persistent_manifold::PersistentManifold,
            shapes::static_plane_shape::StaticPlaneShape,
        },
        dynamics::{rigid_body::RigidBody, sphere_rigid_body::SphereRigidBody},
    },
};

pub fn process_collision(
    convex_obj: &SphereRigidBody,
    plane_obj: &RigidBody,
    plane_shape: &StaticPlaneShape,
    contact_added_callback: &mut ArenaContactTracker,
) -> Option<PersistentManifold> {
    let convex_trans = convex_obj.get_world_trans();
    let convex_shape = convex_obj.get_collision_shape();
    let convex_aabb = convex_shape.get_aabb(convex_trans);
    if !convex_aabb.intersects(&plane_shape.aabb_cache) {
        return None;
    }

    let plane_normal = plane_shape.get_plane_normal();

    let plane_trans = plane_obj.get_world_trans();
    let convex_in_plane_trans = convex_trans - plane_trans;

    let vtx = convex_shape.local_get_supporting_vertex(-plane_normal);
    let vtx_in_plane = convex_in_plane_trans + vtx;
    let distance = plane_normal.dot(vtx_in_plane);

    let contact_breaking_threshold = convex_shape.get_contact_breaking_threshold();
    if distance >= contact_breaking_threshold {
        return None;
    }

    let mut manifold = PersistentManifold::new(contact_breaking_threshold);

    let vtx_in_plane_projected = vtx_in_plane - distance * plane_normal;
    let vtx_in_plane_world = plane_trans + vtx_in_plane_projected;
    let normal_on_surface_b = plane_normal;

    manifold.add_contact_point(
        convex_obj,
        plane_obj,
        normal_on_surface_b,
        vtx_in_plane_world,
        distance,
        None,
        contact_added_callback,
    );

    manifold.refresh_contact_points(convex_obj, plane_obj);
    Some(manifold)
}
