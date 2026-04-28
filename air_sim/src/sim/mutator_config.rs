use glam::Vec3A;

use crate::{GameMode, sim::consts};

#[derive(Clone, Copy, Debug)]
pub struct MutatorConfig {
    pub gravity: Vec3A,
    pub car_mass: f32,
    pub jump_accel: f32,
    pub jump_immediate_force: f32,
    pub boost_accel_air: f32,
    pub boost_used_per_second: f32,
    pub car_max_boost_amount: f32,
    pub car_spawn_boost_amount: f32,

    pub unlimited_flips: bool,
    pub unlimited_double_jumps: bool,
    pub recharge_boost_enabled: bool,
    pub recharge_boost_per_second: f32,
    pub recharge_boost_delay: f32,
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
            car_mass: consts::car::MASS_BT,
            jump_accel: consts::car::jump::ACCEL,
            jump_immediate_force: consts::car::jump::IMMEDIATE_FORCE,
            boost_accel_air: consts::car::boost::ACCEL_AIR,
            boost_used_per_second: match game_mode {
                GameMode::Heatseeker => 0.0,
                _ => consts::car::boost::USED_PER_SECOND,
            },
            car_max_boost_amount: consts::car::boost::MAX,
            car_spawn_boost_amount: match game_mode {
                GameMode::Heatseeker | GameMode::Dropshot => 100.,
                _ => consts::car::boost::SPAWN_AMOUNT,
            },
            unlimited_flips: false,
            unlimited_double_jumps: false,
            recharge_boost_enabled: matches!(game_mode, GameMode::Dropshot),
            recharge_boost_per_second: consts::car::boost::RECHARGE_PER_SECOND,
            recharge_boost_delay: consts::car::boost::RECHARGE_DELAY,
        }
    }
}
