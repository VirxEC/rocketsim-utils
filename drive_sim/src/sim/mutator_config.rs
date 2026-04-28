use crate::{GameMode, sim::consts};

#[derive(Clone, Copy, Debug)]
pub struct MutatorConfig {
    pub boost_used_per_second: f32,
    pub car_spawn_boost_amount: f32,
    pub boost_pad_amount_small: f32,
    pub boost_pad_amount_big: f32,
    pub boost_pad_cooldown_big: f32,
    pub boost_pad_cooldown_small: f32,

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
            boost_used_per_second: match game_mode {
                GameMode::Heatseeker => 0.0,
                _ => consts::car::boost::USED_PER_SECOND,
            },
            car_spawn_boost_amount: match game_mode {
                GameMode::Heatseeker | GameMode::Dropshot => 100.,
                _ => consts::car::boost::SPAWN_AMOUNT,
            },
            boost_pad_amount_big: consts::boost_pads::BOOST_AMOUNT_BIG,
            boost_pad_amount_small: consts::boost_pads::BOOST_AMOUNT_SMALL,
            boost_pad_cooldown_big: consts::boost_pads::COOLDOWN_BIG,
            boost_pad_cooldown_small: consts::boost_pads::COOLDOWN_SMALL,
            recharge_boost_enabled: matches!(game_mode, GameMode::Dropshot),
            recharge_boost_per_second: consts::car::boost::RECHARGE_PER_SECOND,
            recharge_boost_delay: consts::car::boost::RECHARGE_DELAY,
        }
    }
}
