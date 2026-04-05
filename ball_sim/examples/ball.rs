//! Simulate the ball bouncing for 6 seconds from various random positions.
//!
//! Re-runs all simulates in the full version of RocketSim v3 to ensure parity and compare performance.

use std::time::Instant;

use ball_sim::{
    Arena, BallState, GameMode,
    consts::ball::{MAX_ANG_SPEED, MAX_SPEED},
    init_from_default,
};
use glam::Vec3A;
use indicatif::ProgressIterator;
use rand::{
    RngExt, SeedableRng,
    distr::{StandardUniform, Uniform},
    rngs::SmallRng,
};
use rocketsim as rs;

fn generate_sequence() -> Vec<(Vec3A, Vec3A, Vec3A)> {
    let mut x_rng = SmallRng::seed_from_u64(0).sample_iter(Uniform::new(-2800.0, 2800.0).unwrap());
    let mut y_rng = SmallRng::seed_from_u64(1).sample_iter(Uniform::new(-4000.0, 4000.0).unwrap());
    let mut z_rng = SmallRng::seed_from_u64(2).sample_iter(Uniform::new(100.0, 1900.0).unwrap());
    let mut vel_dir_rng = SmallRng::seed_from_u64(3).sample_iter(StandardUniform);
    let mut vel_rng = SmallRng::seed_from_u64(4).sample_iter(Uniform::new(0.0, MAX_SPEED).unwrap());
    let mut ang_dir_rng = SmallRng::seed_from_u64(5).sample_iter(StandardUniform);
    let mut ang_vel_rng =
        SmallRng::seed_from_u64(6).sample_iter(Uniform::new(0.0, MAX_ANG_SPEED).unwrap());

    // (0..1)
    (0..50_000)
        .map(|_| {
            (
                Vec3A::new(
                    x_rng.next().unwrap(),
                    y_rng.next().unwrap(),
                    z_rng.next().unwrap(),
                ),
                Vec3A::new(
                    vel_dir_rng.next().unwrap(),
                    vel_dir_rng.next().unwrap(),
                    vel_dir_rng.next().unwrap(),
                )
                .normalize_or_zero()
                    * vel_rng.next().unwrap(),
                Vec3A::new(
                    ang_dir_rng.next().unwrap(),
                    ang_dir_rng.next().unwrap(),
                    ang_dir_rng.next().unwrap(),
                )
                .normalize_or_zero()
                    * ang_vel_rng.next().unwrap(),
            )
        })
        .collect()
}

fn main() {
    const TICKS_PER_SIM: usize = 6 * 120;

    init_from_default(true).unwrap();
    rs::init_from_default(true).unwrap();

    let sequence = generate_sequence();
    let num = sequence.len();
    let mut states = Vec::with_capacity(num);

    let mut arena = Arena::new(GameMode::Soccar);

    let start_time = Instant::now();

    for (pos, vel, ang_vel) in sequence.iter().copied().progress() {
        let mut ball_state = BallState::DEFAULT;
        ball_state.pos = pos;
        ball_state.vel = vel;
        ball_state.ang_vel = ang_vel;

        arena.set_ball_state(ball_state);

        for _ in 0..TICKS_PER_SIM {
            arena.step_tick();
        }

        states.push(*arena.get_ball_state());
    }

    let irl_time_elapsed = start_time.elapsed().as_secs_f64();

    let total_frames = num * TICKS_PER_SIM;
    let sim_time_elapsed = total_frames / 120;

    let speedup = sim_time_elapsed as f64 / irl_time_elapsed;
    let time_per_sim = irl_time_elapsed * 1_000_000.0 / num as f64;
    let ticks_per_second = TICKS_PER_SIM as f64 / time_per_sim * 1_000_000.0;
    println!(
        "Finished simulating {sim_time_elapsed:.2}s in {irl_time_elapsed:.2}s ({speedup:.0}x; {time_per_sim:.1}mus per, {ticks_per_second:.0})tps"
    );

    let mut arena_rs = rs::Arena::new_with_config(
        rs::GameMode::Soccar,
        rs::ArenaConfig {
            no_ball_rot: true,
            ..Default::default()
        },
    );

    let start_time = Instant::now();
    for ((pos, vel, ang_vel), final_state) in sequence.iter().copied().zip(&states).progress() {
        let mut ball_state = rs::BallState::DEFAULT;
        ball_state.pos = pos;
        ball_state.vel = vel;
        ball_state.ang_vel = ang_vel;

        arena_rs.set_ball_state(ball_state);

        for _ in 0..TICKS_PER_SIM {
            arena_rs.step_tick();
        }

        let final_state_rs = arena_rs.get_ball_state().phys;

        let pos_diff = (final_state.pos - final_state_rs.pos).length();
        let vel_diff = (final_state.vel - final_state_rs.vel).length();
        let ang_vel_diff = (final_state.ang_vel - final_state_rs.ang_vel).length();
        assert!(
            pos_diff < 0.01 && vel_diff < 0.01 && ang_vel_diff < 0.001,
            "Final states differ too much: pos_diff={pos_diff}, vel_diff={vel_diff}, ang_vel_diff={ang_vel_diff}"
        );
    }

    let irl_time_elapsed = start_time.elapsed().as_secs_f64();

    println!("All final states match RocketSim's results!");

    let total_frames = num * TICKS_PER_SIM;
    let sim_time_elapsed = total_frames / 120;

    let speedup = sim_time_elapsed as f64 / irl_time_elapsed;
    let time_per_sim = irl_time_elapsed * 1_000_000.0 / num as f64;
    let ticks_per_second = TICKS_PER_SIM as f64 / time_per_sim * 1_000_000.0;
    println!(
        "Finished simulating {sim_time_elapsed:.2}s in {irl_time_elapsed:.2}s ({speedup:.0}x; {time_per_sim:.1}mus per, {ticks_per_second:.0})tps"
    );
}
