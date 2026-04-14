use crate::{
    Car, CarBodyConfig, CarControls, CarInfo, CarState, GameMode, MutatorConfig,
    bullet::dynamics::{
        constraint_solver::seq_impulse_constraint_solver::SeqImpulseConstraintSolver,
        discrete_dynamics_world::DiscreteDynamicsWorld,
    },
    consts::{TICK_TIME, UU_TO_BT},
};

pub struct Arena {
    pub(crate) bullet_world: DiscreteDynamicsWorld,

    pub(crate) cars: Vec<Car>,
    pub(crate) tick_count: u64,
    pub(crate) mutator_config: MutatorConfig,
}

impl Arena {
    pub fn new(game_mode: GameMode) -> Self {
        Self::new_with_config(MutatorConfig::new(game_mode))
    }

    pub fn new_with_config(mutator_config: MutatorConfig) -> Self {
        let constraint_solver = SeqImpulseConstraintSolver::default();

        let mut bullet_world = DiscreteDynamicsWorld::new(constraint_solver);
        bullet_world.set_gravity(mutator_config.gravity * UU_TO_BT);

        Self {
            mutator_config,
            tick_count: 0,
            cars: Vec::with_capacity(6),
            bullet_world,
        }
    }

    /// Creates and adds a car to the arena, returning the index of the car in the cars vector
    pub fn add_car(&mut self, config: CarBodyConfig) -> usize {
        let idx = self.cars.len();
        let car = Car::new(idx, &mut self.bullet_world, &self.mutator_config, config);

        self.cars.push(car);
        idx
    }

    /// Steps the arena for 1 tick, returning the events produced during that tick
    pub fn step_tick(&mut self) {
        for car in &mut self.cars {
            car.pre_tick_update(&mut self.bullet_world, &self.mutator_config);
        }

        self.bullet_world.step_simulation(TICK_TIME);

        for car in &mut self.cars {
            let rb = &mut self.bullet_world.bodies_mut()[car.rigid_body_idx];
            car.post_tick_update(rb);
            car.finish_physics_tick(rb);
        }

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
    pub const fn cars(&self) -> &Vec<Car> {
        &self.cars
    }

    #[inline]
    pub const fn num_cars(&self) -> usize {
        self.cars.len()
    }

    pub fn get_car_info(&self, car_idx: usize) -> &CarInfo {
        &self.cars[car_idx].info
    }

    pub fn get_car_state(&self, car_idx: usize) -> &CarState {
        self.cars[car_idx].get_state()
    }

    pub fn get_car_controls(&self, car_idx: usize) -> &CarControls {
        &self.cars[car_idx].state.controls
    }

    pub fn get_car_info_and_state(&self, car_idx: usize) -> (&CarInfo, &CarState) {
        let car = &self.cars[car_idx];
        (&car.info, &car.state)
    }

    pub fn set_car_state(&mut self, car_idx: usize, state: CarState) {
        let car = &mut self.cars[car_idx];

        car.set_state(
            &mut self.bullet_world.bodies_mut()[car.rigid_body_idx],
            &state,
        );
    }

    pub fn set_car_controls(&mut self, car_idx: usize, controls: CarControls) {
        self.cars[car_idx].state.controls = controls
    }
}
