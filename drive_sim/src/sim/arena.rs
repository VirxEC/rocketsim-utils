use crate::{
    BoostPadConfig, BoostPadGrid, BoostPadState, Car, CarBodyConfig, CarControls, CarState,
    GameMode, MutatorConfig,
    bullet::dynamics::discrete_dynamics_world::DiscreteDynamicsWorld,
    consts::{self, TICK_RATE, TICK_TIME},
    sim::BoostPad,
};

pub struct Arena {
    bullet_world: DiscreteDynamicsWorld,
    car: Car,
    tick_count: u64,
    game_mode: GameMode,
    mutator_config: MutatorConfig,
    boost_pad_grid: Option<BoostPadGrid>,
}

impl Arena {
    pub fn new(game_mode: GameMode) -> Self {
        let mutator_config = MutatorConfig::new(game_mode);

        let (car, car_body) = Car::new(&mutator_config, CarBodyConfig::OCTANE);
        let bullet_world = DiscreteDynamicsWorld {
            collision_obj: car_body,
        };

        let mut boost_pad_grid = None;
        if game_mode != GameMode::Dropshot {
            let mut boost_pad_configs = Vec::new();

            let small_pad_locs = consts::boost_pads::get_locations(game_mode, false);
            let big_pad_locs = consts::boost_pads::get_locations(game_mode, true);
            boost_pad_configs.reserve(small_pad_locs.len() + big_pad_locs.len());

            for small_pos in small_pad_locs {
                boost_pad_configs.push(BoostPadConfig {
                    pos: *small_pos,
                    is_big: false,
                });
            }

            for big_pos in big_pad_locs {
                boost_pad_configs.push(BoostPadConfig {
                    pos: *big_pos,
                    is_big: true,
                });
            }

            boost_pad_grid = Some(BoostPadGrid::new(&boost_pad_configs, &mutator_config));
        }

        Self {
            game_mode,
            boost_pad_grid,
            mutator_config,
            tick_count: 0,
            car,
            bullet_world,
        }
    }

    /// Steps the arena for 1 tick, returning the events produced during that tick
    pub fn step_tick(&mut self) {
        self.car
            .pre_tick_update(&mut self.bullet_world, &self.mutator_config);

        self.bullet_world.step_simulation();

        self.car
            .finish_physics_tick(&mut self.bullet_world.collision_obj);

        if let Some(boost_pad_grid) = self.boost_pad_grid.as_mut() {
            boost_pad_grid.maybe_give_car_boost(&mut self.car.state, self.tick_count);
        }

        self.tick_count += 1;
    }

    #[inline]
    pub const fn tick_count(&self) -> u64 {
        self.tick_count
    }

    #[inline]
    pub const fn game_mode(&self) -> GameMode {
        self.game_mode
    }

    #[inline]
    pub const fn mutator_config(&self) -> &MutatorConfig {
        &self.mutator_config
    }

    #[inline]
    pub const fn car(&self) -> &Car {
        &self.car
    }

    pub fn get_car_state(&self) -> &CarState {
        self.car.get_state()
    }

    pub fn get_car_controls(&self) -> &CarControls {
        &self.car.state.controls
    }

    pub fn set_car_state(&mut self, state: CarState) {
        self.car
            .set_state(&mut self.bullet_world.collision_obj, &state);
    }

    pub fn set_car_controls(&mut self, controls: CarControls) {
        self.car.state.controls = controls;
    }

    pub fn get_boost_pad_state(&self, idx: usize) -> BoostPadState {
        let pad = self.boost_pads()[idx];

        let cooldown = if let Some(gave_boost_tick) = pad.gave_boost_tick_count {
            let max_cooldown = pad.max_cooldown;
            let time_since = ((self.tick_count() - gave_boost_tick) as f32) * TICK_TIME;
            (max_cooldown - time_since).max(0.0)
        } else {
            0.0
        };

        BoostPadState { cooldown }
    }

    pub fn set_boost_pad_state(&mut self, idx: usize, state: BoostPadState) {
        let boost_pad_grid = self.boost_pad_grid.as_mut().unwrap();
        let tick_count = self.tick_count;
        let pad = &mut boost_pad_grid.all_pads[idx];
        if state.cooldown > 0.0 {
            let time_since_pickup = (pad.max_cooldown - state.cooldown).max(0.0);
            let ticks_since_pickup = (time_since_pickup * TICK_RATE).round() as u64;
            pad.gave_boost_tick_count = Some(tick_count - ticks_since_pickup);
        } else {
            boost_pad_grid.all_pads[idx].gave_boost_tick_count = None;
        }
    }

    pub fn get_boost_pad_config(&self, idx: usize) -> &BoostPadConfig {
        self.boost_pads()[idx].config()
    }

    pub(crate) fn boost_pads(&self) -> &[BoostPad] {
        &self.boost_pad_grid.as_ref().unwrap().all_pads
    }

    pub fn num_boost_pads(&self) -> usize {
        self.boost_pads().len()
    }
}
