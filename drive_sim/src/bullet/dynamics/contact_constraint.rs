use glam::Vec3A;

use crate::bullet::dynamics::rigid_body::RigidBody;

pub fn resolve_single_bilateral(chassis: &RigidBody, pos: Vec3A, normal: Vec3A) -> f32 {
    const CONTACT_DAMPING: f32 = -0.2;

    debug_assert!(normal.is_normalized());
    let chassis_comt = chassis.get_world_trans();
    let rel_pos1 = pos - chassis_comt.translation;
    let vel = chassis.get_vel_in_local_point(rel_pos1);

    let a_j = chassis_comt
        .matrix3
        .mul_transpose_vec3a(rel_pos1.cross(normal));
    let min_v_jt_0 = chassis.inv_inertia_local * a_j;

    let jac_diag_ab = chassis.inverse_mass + min_v_jt_0.dot(a_j);
    let rel_vel = normal.dot(vel);

    CONTACT_DAMPING * rel_vel / jac_diag_ab
}
