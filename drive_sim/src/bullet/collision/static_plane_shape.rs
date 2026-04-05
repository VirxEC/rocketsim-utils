use glam::{Vec3A, Vec4};

pub struct StaticPlaneShape;

fn to_simd(vecs: &[Vec3A; 4]) -> [Vec4; 3] {
    [
        Vec4::from_array([vecs[0].x, vecs[1].x, vecs[2].x, vecs[3].x]),
        Vec4::from_array([vecs[0].y, vecs[1].y, vecs[2].y, vecs[3].y]),
        Vec4::from_array([vecs[0].z, vecs[1].z, vecs[2].z, vecs[3].z]),
    ]
}

fn from_simd(vecs: &[Vec4; 3]) -> [Vec3A; 4] {
    [
        Vec3A::new(vecs[0].x, vecs[1].x, vecs[2].x),
        Vec3A::new(vecs[0].y, vecs[1].y, vecs[2].y),
        Vec3A::new(vecs[0].z, vecs[1].z, vecs[2].z),
        Vec3A::new(vecs[0].w, vecs[1].w, vecs[2].w),
    ]
}

impl StaticPlaneShape {
    /// We assume that all rays will connect with the floor.
    /// Performs all 4 ray casts at once, using SIMD.
    ///
    /// Returns the hit points in world space.
    pub fn perform_raycast(sources: &[Vec3A; 4], targets: &[Vec3A; 4]) -> [Vec3A; 4] {
        let sources = to_simd(sources);
        let targets = to_simd(targets);

        let delta = [
            targets[0] - sources[0],
            targets[1] - sources[1],
            targets[2] - sources[2],
        ];

        let dist = (delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]).sqrt();
        let hit_fractions = sources[2] / dist;

        let s = Vec4::ONE - hit_fractions;
        let hit_point_in_world = [
            s * sources[0] + hit_fractions * targets[0],
            s * sources[1] + hit_fractions * targets[1],
            s * sources[2] + hit_fractions * targets[2],
        ];

        from_simd(&hit_point_in_world)
    }
}
