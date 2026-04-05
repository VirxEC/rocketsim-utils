use glam::Vec3A;

#[derive(Clone, Copy, Debug, Default, Hash, PartialEq, Eq)]
pub enum ArenaMemWeightMode {
    #[default]
    Heavy,
    Light,
}

#[derive(Clone, Debug)]
pub struct ArenaConfig {
    pub mem_weight_mode: ArenaMemWeightMode,
    pub min_pos: Vec3A,
    pub max_pos: Vec3A,
    pub max_aabb_len: f32,
}

impl Default for ArenaConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl ArenaConfig {
    pub const DEFAULT: Self = Self {
        mem_weight_mode: ArenaMemWeightMode::Heavy,
        min_pos: Vec3A::new(-5600., -6000., 0.),
        max_pos: Vec3A::new(5600., 6000., 2200.),
        max_aabb_len: 250.,
    };
}
