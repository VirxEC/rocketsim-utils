use std::{path::PathBuf, sync::OnceLock};

use ball_sim::{Arena, BallState, GameMode, init};
use glam::{Mat3A, Vec3A};
use rand::{RngExt, SeedableRng, rngs::SmallRng};
use rocketsim as rs;

const POS_TOL: f32 = 0.01;
const VEL_TOL: f32 = 0.01;
const ANG_VEL_TOL: f32 = 0.001;

fn ensure_init() {
    static INIT: OnceLock<()> = OnceLock::new();
    INIT.get_or_init(|| {
        let dir: PathBuf = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../collision_meshes");
        init(&dir, true).unwrap();
        rs::init(&dir, true).unwrap();
    });
}

const fn rs_mode(mode: GameMode) -> rs::GameMode {
    match mode {
        GameMode::Soccar => rs::GameMode::Soccar,
        GameMode::Hoops => rs::GameMode::Hoops,
        GameMode::Heatseeker => rs::GameMode::Heatseeker,
        GameMode::Snowday => rs::GameMode::Snowday,
        GameMode::Dropshot => rs::GameMode::Dropshot,
        GameMode::TheVoid => rs::GameMode::TheVoid,
    }
}

fn to_rs_state(state: &BallState) -> rs::BallState {
    rs::BallState {
        phys: rs::PhysState {
            pos: state.phys.pos,
            rot_mat: Mat3A::IDENTITY,
            vel: state.phys.vel,
            ang_vel: state.phys.ang_vel,
        },
        hs_info: rs::HeatseekerInfo {
            y_target_dir: state.hs_info.y_target_dir,
            cur_target_speed: state.hs_info.cur_target_speed,
            time_since_hit: state.hs_info.time_since_hit,
        },
        ds_info: rs::DropshotInfo {
            charge_level: state.ds_info.charge_level,
            accumulated_hit_force: state.ds_info.accumulated_hit_force,
            y_target_dir: state.ds_info.y_target_dir,
            last_damage_tick: state.ds_info.last_damage_tick,
        },
        tick_count_since_kickoff: state.tick_count_since_kickoff,
    }
}

fn new_arenas(mode: GameMode) -> (Arena, rs::Arena) {
    let arena = Arena::new(mode);
    let rs_mode = rs_mode(mode);
    let arena_rs = rs::Arena::new_with_config(rs::ArenaConfig {
        game_mode: rs_mode,
        mutators: rs::MutatorConfig::new(rs_mode),
        no_ball_rot: true,
        ..Default::default()
    });
    (arena, arena_rs)
}

fn assert_parity(mode: GameMode, state: BallState, ticks: usize) {
    let (mut arena, mut arena_rs) = new_arenas(mode);
    arena.set_ball_state(state);
    arena_rs.set_ball_state(to_rs_state(&state));

    for _ in 0..ticks {
        arena.step_tick();
        arena_rs.step_tick();
    }

    let got = arena.get_ball_state();
    let want = arena_rs.get_ball_state().phys;

    let pos_diff = (got.pos - want.pos).length();
    let vel_diff = (got.vel - want.vel).length();
    let ang_vel_diff = (got.ang_vel - want.ang_vel).length();
    assert!(
        pos_diff < POS_TOL && vel_diff < VEL_TOL && ang_vel_diff < ANG_VEL_TOL,
        "{mode:?} states differ too much after {ticks} ticks: \
         pos_diff={pos_diff}, vel_diff={vel_diff}, ang_vel_diff={ang_vel_diff} \
         (start pos={} vel={} ang_vel={})",
        state.pos,
        state.vel,
        state.ang_vel,
    );
}

fn random_state(
    rng: &mut SmallRng,
    x_max: f32,
    y_max: f32,
    z_max: f32,
    max_speed: f32,
) -> BallState {
    let pos = Vec3A::new(
        rng.random_range(-x_max..x_max),
        rng.random_range(-y_max..y_max),
        rng.random_range(100.0..z_max),
    );
    let vel = Vec3A::new(
        rng.random_range(-1.0..1.0),
        rng.random_range(-1.0..1.0),
        rng.random_range(-1.0..1.0),
    )
    .normalize_or_zero()
        * rng.random_range(0.0..max_speed);
    let ang_vel = Vec3A::new(
        rng.random_range(-1.0..1.0),
        rng.random_range(-1.0..1.0),
        rng.random_range(-1.0..1.0),
    )
    .normalize_or_zero()
        * rng.random_range(0.0..ball_sim::consts::ball::MAX_ANG_SPEED);
    BallState {
        phys: ball_sim::PhysState { pos, vel, ang_vel },
        ..BallState::DEFAULT
    }
}

#[test]
fn soccar_random_bounces() {
    ensure_init();
    let mut rng = SmallRng::seed_from_u64(0x50CC4A);
    for _ in 0..32 {
        assert_parity(
            GameMode::Soccar,
            random_state(&mut rng, 2800.0, 4000.0, 1900.0, 6000.0),
            720,
        );
    }
}

#[test]
fn soccar_high_speed_clamp() {
    ensure_init();
    let mut state = BallState::DEFAULT;
    state.pos = Vec3A::new(500.0, -1000.0, 800.0);
    state.vel = Vec3A::new(3000.0, -5000.0, 6000.0);
    state.ang_vel = Vec3A::new(0.0, 0.0, 20.0);
    assert_parity(GameMode::Soccar, state, 720);
}

#[test]
fn soccar_targeted_contacts() {
    ensure_init();
    let cases = [
        (
            Vec3A::new(3900.0, 0.0, 500.0),
            Vec3A::new(2500.0, 0.0, 0.0),
            Vec3A::ZERO,
        ),
        (
            Vec3A::new(0.0, 0.0, 500.0),
            Vec3A::new(0.0, 0.0, -3000.0),
            Vec3A::new(3.0, 0.0, 0.0),
        ),
        (
            Vec3A::new(0.0, 4900.0, 400.0),
            Vec3A::new(0.0, 1500.0, 800.0),
            Vec3A::ZERO,
        ),
        (
            Vec3A::new(3900.0, 4900.0, 1500.0),
            Vec3A::new(1000.0, 1000.0, -1000.0),
            Vec3A::new(0.0, 4.0, 2.0),
        ),
        (
            Vec3A::new(-1000.0, 2000.0, 300.0),
            Vec3A::new(-3000.0, 2000.0, 1000.0),
            Vec3A::new(1.0, 2.0, 3.0),
        ),
        (
            Vec3A::new(0.0, 0.0, 2000.0),
            Vec3A::ZERO,
            Vec3A::new(6.0, 6.0, 6.0),
        ),
    ];
    for (pos, vel, ang_vel) in cases {
        let state = BallState {
            phys: ball_sim::PhysState { pos, vel, ang_vel },
            ..BallState::DEFAULT
        };
        assert_parity(GameMode::Soccar, state, 360);
    }
}

#[test]
fn hoops_kickoff_launch_and_bounces() {
    ensure_init();
    let (mut arena, mut arena_rs) = new_arenas(GameMode::Hoops);
    arena.set_ball_state(BallState::DEFAULT);
    arena_rs.set_ball_state(to_rs_state(&BallState::DEFAULT));
    for _ in 0..120 {
        arena.step_tick();
        arena_rs.step_tick();
    }
    assert!(arena.get_ball_state().pos.z > 500.0);
    assert!(arena_rs.get_ball_state().phys.pos.z > 500.0);
    assert_parity(GameMode::Hoops, BallState::DEFAULT, 120);

    let mut rng = SmallRng::seed_from_u64(0x60095);
    for _ in 0..8 {
        assert_parity(
            GameMode::Hoops,
            random_state(&mut rng, 2500.0, 3000.0, 1700.0, 6000.0),
            360,
        );
    }
}

#[test]
fn heatseeker_seek_parity() {
    ensure_init();
    let mut state = BallState::DEFAULT;
    state.pos = Vec3A::new(800.0, -1500.0, 400.0);
    state.vel = Vec3A::new(0.0, -65.0, 650.0);
    state.hs_info.y_target_dir = 1;
    state.hs_info.cur_target_speed = 2985.0;
    assert_parity(GameMode::Heatseeker, state, 360);

    let mut arena = Arena::new(GameMode::Heatseeker);
    arena.reset_to_kickoff(ball_sim::Team::Blue);
    let blue = *arena.get_ball_state();
    arena.reset_to_kickoff(ball_sim::Team::Orange);
    let orange = *arena.get_ball_state();
    assert_eq!(blue.pos.x, ball_sim::consts::heatseeker::BALL_START_POS.x);
    assert_eq!(blue.pos.y, -ball_sim::consts::heatseeker::BALL_START_POS.y);
    assert_eq!(orange.pos.y, -blue.pos.y);
    assert_eq!(orange.vel.y, -blue.vel.y);
}
