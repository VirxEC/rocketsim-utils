use glam::{Vec3A, Vec3Swizzles};

const CONVEX_DISTANCE_MARGIN: f32 = 0.04;

pub fn calculate_local_intertia(box_half_extents: Vec3A, mass: f32) -> Vec3A {
    let l = 2.0 * (box_half_extents - CONVEX_DISTANCE_MARGIN);
    let yxx = l.yxx();
    let zzy = l.zzy();

    mass / 12.0 * (yxx * yxx + zzy * zzy)
}
