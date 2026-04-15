use crate::{Car, CarBodyConfig, CarControls, CarState, GameMode, MutatorConfig, consts::UU_TO_BT};

pub struct Arena {
    car: Car,
    mutator_config: MutatorConfig,
}

impl Arena {
    pub fn new(game_mode: GameMode) -> Self {
        Self::new_with_config(MutatorConfig::new(game_mode), CarBodyConfig::OCTANE, 120)
    }

    pub fn new_with_config(
        mutator_config: MutatorConfig,
        car_body: CarBodyConfig,
        tick_rate: u8,
    ) -> Self {
        assert!(
            (30..=120).contains(&tick_rate),
            "Tick rate must be between 30 and 120"
        );

        let car = Car::new(
            &mutator_config,
            mutator_config.gravity * UU_TO_BT,
            car_body,
            f32::from(tick_rate),
        );

        Self {
            mutator_config,
            car,
        }
    }

    /// Steps the arena for 1 tick, returning the events produced during that tick
    pub fn step_tick(&mut self) {
        self.car.step_tick(&self.mutator_config);
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

    pub fn set_car_state(&mut self, state: CarState) {
        self.car.set_state(&state);
    }

    pub fn set_car_controls(&mut self, controls: CarControls) {
        self.car.set_controls(controls);
    }
}
