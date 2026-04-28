use glam::Vec3A;

use crate::{GameMode, sim::consts};

#[derive(Clone, Copy, Debug, Default, Hash, PartialEq, Eq)]
pub enum DemoMode {
    #[default]
    Normal,
    OnContact,
    Disabled,
}

#[derive(Clone, Copy, Debug)]
pub struct MutatorConfig {
    pub gravity: Vec3A,
    pub ball_mass: f32,
    pub ball_max_speed: f32,
    pub ball_drag: f32,
    pub ball_hit_extra_force_scale: f32,
    pub bump_force_scale: f32,
    pub bump_requires_front_hit: bool,
    pub ball_radius: f32,
    /// Only used if the game mode has soccar goals (i.e. soccar, heatseeker, snowday)
    pub goal_base_threshold_y: f32,
}

impl Default for MutatorConfig {
    fn default() -> Self {
        const { Self::new(GameMode::Soccar) }
    }
}

impl MutatorConfig {
    #[must_use]
    pub const fn new(game_mode: GameMode) -> Self {
        Self {
            gravity: Vec3A::new(0., 0., consts::GRAVITY_Z),
            ball_mass: if matches!(game_mode, GameMode::Snowday) {
                consts::snowday::PUCK_MASS_BT
            } else {
                consts::ball::MASS_BT
            },
            ball_max_speed: consts::ball::MAX_SPEED,
            ball_drag: consts::ball::DRAG,
            ball_hit_extra_force_scale: 1.,
            bump_force_scale: 1.,
            bump_requires_front_hit: false, // No longer required in newer Rocket League versions
            ball_radius: consts::ball::get_radius(game_mode),
            goal_base_threshold_y: consts::goal::SOCCAR_GOAL_SCORE_BASE_THRESHOLD_Y,
        }
    }
}
