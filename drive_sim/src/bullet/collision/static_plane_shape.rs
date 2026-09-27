use glam::Vec4;

pub struct StaticPlaneShape;

impl StaticPlaneShape {
    pub fn perform_raycast(sources: &[Vec4; 3], targets: &[Vec4; 3]) -> [Vec4; 3] {
        let delta = [
            targets[0] - sources[0],
            targets[1] - sources[1],
            targets[2] - sources[2],
        ];

        let dist = (delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]).sqrt();
        let hit_fractions = sources[2] / dist;
        let s = Vec4::ONE - hit_fractions;

        [
            s * sources[0] + hit_fractions * targets[0],
            s * sources[1] + hit_fractions * targets[1],
            s * sources[2] + hit_fractions * targets[2],
        ]
    }
}
