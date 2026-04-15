use crate::{
    Car, CarBodyConfig, CarControls, CarState, GameMode, MutatorConfig,
    bullet::dynamics::{
        constraint_solver::seq_impulse_constraint_solver::SeqImpulseConstraintSolver,
        discrete_dynamics_world::DiscreteDynamicsWorld,
    },
    consts::{TICK_TIME, UU_TO_BT},
};

pub struct Arena {
    pub(crate) bullet_world: DiscreteDynamicsWorld,
    pub(crate) car: Car,
    pub(crate) tick_count: u64,
    pub(crate) mutator_config: MutatorConfig,
}

impl Arena {
    pub fn new(game_mode: GameMode) -> Self {
        Self::new_with_config(MutatorConfig::new(game_mode), CarBodyConfig::OCTANE)
    }

    pub fn new_with_config(mutator_config: MutatorConfig, car_body: CarBodyConfig) -> Self {
        let constraint_solver = SeqImpulseConstraintSolver::default();

        let (car, body) = Car::new(&mutator_config, car_body);
        let mut bullet_world = DiscreteDynamicsWorld::new(constraint_solver, body);
        bullet_world.set_gravity(mutator_config.gravity * UU_TO_BT);

        Self {
            mutator_config,
            tick_count: 0,
            car,
            bullet_world,
        }
    }

    /// Steps the arena for 1 tick, returning the events produced during that tick
    pub fn step_tick(&mut self) {
        self.car
            .pre_tick_update(self.bullet_world.body_mut(), &self.mutator_config);

        self.bullet_world.step_simulation(TICK_TIME);

        self.car.post_tick_update(self.bullet_world.body_mut());
        self.car.finish_physics_tick(self.bullet_world.body_mut());

        self.tick_count += 1;
    }

    #[inline]
    pub const fn tick_count(&self) -> u64 {
        self.tick_count
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

    pub fn get_car_config(&self) -> &CarBodyConfig {
        &self.car.config
    }

    pub fn get_car_controls(&self) -> &CarControls {
        &self.car.state.controls
    }

    pub fn set_car_state(&mut self, state: CarState) {
        self.car.set_state(self.bullet_world.body_mut(), &state);
    }

    pub fn set_car_controls(&mut self, controls: CarControls) {
        self.car.state.controls = controls
    }
}
