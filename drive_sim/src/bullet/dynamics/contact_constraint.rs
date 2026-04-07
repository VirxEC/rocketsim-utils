use glam::Vec4;

use crate::bullet::dynamics::rigid_body::RigidBody;

pub fn resolve_single_bilateral_simd(
    chassis: &RigidBody,
    rel_pos_x: Vec4,
    rel_pos_y: Vec4,
    rel_pos_z: Vec4,
    normal_x: Vec4,
    normal_y: Vec4,
    normal_z: Vec4,
) -> Vec4 {
    const CONTACT_DAMPING: Vec4 = Vec4::splat(-0.2);

    let avx = Vec4::splat(chassis.ang_vel.x);
    let avy = Vec4::splat(chassis.ang_vel.y);
    let avz = Vec4::splat(chassis.ang_vel.z);

    let cross_x = avy * rel_pos_z - avz * rel_pos_y;
    let cross_y = avz * rel_pos_x - avx * rel_pos_z;
    let cross_z = avx * rel_pos_y - avy * rel_pos_x;

    let vel_x = cross_x + chassis.lin_vel.x;
    let vel_y = cross_y + chassis.lin_vel.y;
    let vel_z = cross_z + chassis.lin_vel.z;

    let rel_vel = normal_x * vel_x + normal_y * vel_y + normal_z * vel_z;

    let c_x = rel_pos_y * normal_z - rel_pos_z * normal_y;
    let c_y = rel_pos_z * normal_x - rel_pos_x * normal_z;
    let c_z = rel_pos_x * normal_y - rel_pos_y * normal_x;

    let chassis_comt = chassis.get_world_trans();
    let ax = chassis_comt.matrix3.x_axis;
    let ay = chassis_comt.matrix3.y_axis;
    let az = chassis_comt.matrix3.z_axis;

    let a_j_x = c_x * ax.x + c_y * ax.y + c_z * ax.z;
    let a_j_y = c_x * ay.x + c_y * ay.y + c_z * ay.z;
    let a_j_z = c_x * az.x + c_y * az.y + c_z * az.z;

    let inv = chassis.inv_inertia_local;
    let min_v_jt_0_x = a_j_x * inv.x;
    let min_v_jt_0_y = a_j_y * inv.y;
    let min_v_jt_0_z = a_j_z * inv.z;

    let jac_diag_ab =
        chassis.inverse_mass + min_v_jt_0_x * a_j_x + min_v_jt_0_y * a_j_y + min_v_jt_0_z * a_j_z;

    CONTACT_DAMPING * rel_vel / jac_diag_ab
}
