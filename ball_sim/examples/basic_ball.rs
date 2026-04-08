use ball_sim::{Arena, BallState, GameMode, init_from_default};
use glam::Vec3A;

fn main() {
    // Must be ran once _per process_
    init_from_default(true).unwrap();

    // Can be created at will
    let mut arena = Arena::new(GameMode::Soccar);

    // Set the initial ball configuration
    let mut ball_state = BallState::DEFAULT;
    ball_state.pos = Vec3A::new(0.0, 0.0, 100.0);
    ball_state.vel = Vec3A::new(200.0, 100.0, 500.0);
    ball_state.ang_vel = Vec3A::new(1.0, 2.0, 3.0);
    arena.set_ball_state(ball_state);

    // Step the simulation forward by one tick
    arena.step_tick();
    arena.step_tick();

    // Get the current ball state
    let ball_state = arena.get_ball_state();
    dbg!(ball_state.pos, ball_state.vel, ball_state.ang_vel);
}
