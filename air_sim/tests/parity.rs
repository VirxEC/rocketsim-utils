//! Parity tests against the full version of RocketSim v3.
//!
//! These run aerial scenarios in both this crate's `Arena` and RocketSim's
//! `Arena` side-by-side, asserting that the car states stay in sync.

use std::path::Path;

use air_sim::{Arena as SimArena, CarControls, CarState, GameMode};
use glam::Vec3A;
use rocketsim as rs;

fn init_rs() {
    let path = if Path::new("collision_meshes").exists() {
        "./collision_meshes/"
    } else {
        "../collision_meshes/"
    };
    rs::init(path, true).unwrap();
}

fn make_rs_arena() -> (rs::Arena, usize) {
    let mut arena = rs::Arena::new_with_config(rs::ArenaConfig {
        game_mode: rs::GameMode::Soccar,
        ..Default::default()
    });
    let car_idx = arena.add_car(rs::Team::Blue, rs::CarBodyConfig::OCTANE);
    (arena, car_idx)
}

/// Converts a sim `CarState` into a rocketsim `CarState`.
fn to_rs_state(state: &CarState) -> rs::CarState {
    rs::CarState {
        phys: rs::PhysState {
            pos: state.phys.pos,
            rot_mat: state.phys.rot_mat,
            vel: state.phys.vel,
            ang_vel: state.phys.ang_vel,
        },
        has_jumped: state.has_jumped,
        is_jumping: state.is_jumping,
        air_time_since_jump: state.air_time_since_jump,
        boost: state.boost,
        // The sim is aerial-only; make sure rocketsim doesn't treat the car
        // as grounded on the first tick
        is_on_ground: false,
        ..rs::CarState::default()
    }
}

const POS_TOL: f32 = 0.5; // uu
const VEL_TOL: f32 = 0.5; // uu/s
const ANG_VEL_TOL: f32 = 0.05; // radians/s

#[track_caller]
fn assert_states_close(sim_state: &CarState, rs_state: &rs::CarState, tick: usize) {
    let pos_diff = (sim_state.phys.pos - rs_state.phys.pos).length();
    let vel_diff = (sim_state.phys.vel - rs_state.phys.vel).length();
    let ang_vel_diff = (sim_state.phys.ang_vel - rs_state.phys.ang_vel).length();
    assert!(
        pos_diff < POS_TOL && vel_diff < VEL_TOL && ang_vel_diff < ANG_VEL_TOL,
        "states diverged at tick {tick}: \
         pos_diff={pos_diff}, vel_diff={vel_diff}, ang_vel_diff={ang_vel_diff}\n\
         sim:  pos={} vel={} ang={}\n\
         rs:   pos={} vel={} ang={}",
        sim_state.phys.pos,
        sim_state.phys.vel,
        sim_state.phys.ang_vel,
        rs_state.phys.pos,
        rs_state.phys.vel,
        rs_state.phys.ang_vel
    );
}

#[test]
fn aerial_free_flight_parity() {
    init_rs();

    let mut sim = SimArena::new(GameMode::Soccar);
    let (mut rs_arena, car_idx) = make_rs_arena();

    // Spawn both cars mid-air with some horizontal velocity
    let mut sim_state = CarState::DEFAULT;
    sim_state.phys.pos = Vec3A::new(0.0, 0.0, 500.0);
    sim_state.phys.vel = Vec3A::new(400.0, 0.0, 0.0);
    sim.set_car_state(sim_state);

    rs_arena.set_car_state(car_idx, to_rs_state(&sim_state));

    for tick in 0..130 {
        let mut controls = CarControls::DEFAULT;
        match tick {
            0..=49 => controls.pitch = 1.0,
            50..=99 => {
                controls.yaw = 1.0;
                controls.roll = -1.0;
            }
            100..=129 => controls.pitch = -1.0,
            _ => {}
        }

        let rs_controls = rs::CarControls {
            throttle: controls.throttle,
            pitch: controls.pitch,
            yaw: controls.yaw,
            roll: controls.roll,
            jump: controls.jump,
            boost: controls.boost,
            ..rs::CarControls::DEFAULT
        };
        sim.set_car_controls(controls);
        rs_arena.set_car_controls(car_idx, rs_controls);

        sim.step_tick();
        rs_arena.step_tick();

        assert_states_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);
    }
}

#[test]
fn aerial_dodge_parity() {
    init_rs();

    let mut sim = SimArena::new(GameMode::Soccar);
    let (mut rs_arena, car_idx) = make_rs_arena();

    // Spawn both cars mid-air, moving forward, having already jumped so a
    // second jump input triggers a flip
    let mut sim_state = CarState::DEFAULT;
    sim_state.phys.pos = Vec3A::new(0.0, 0.0, 500.0);
    sim_state.phys.vel = Vec3A::new(900.0, 0.0, 0.0);
    sim_state.has_jumped = true;
    sim_state.is_jumping = false;
    sim_state.air_time_since_jump = 0.2;
    sim.set_car_state(sim_state);

    rs_arena.set_car_state(car_idx, to_rs_state(&sim_state));

    for tick in 0..120 {
        // On tick 10, dodge sideways
        let mut controls = CarControls::DEFAULT;
        if tick == 10 {
            controls.jump = true;
            controls.yaw = 1.0;
        }

        let rs_controls = rs::CarControls {
            throttle: controls.throttle,
            pitch: controls.pitch,
            yaw: controls.yaw,
            roll: controls.roll,
            jump: controls.jump,
            boost: controls.boost,
            ..rs::CarControls::DEFAULT
        };
        sim.set_car_controls(controls);
        rs_arena.set_car_controls(car_idx, rs_controls);

        sim.step_tick();
        rs_arena.step_tick();

        assert_states_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);

        // The flip must be active on both sides at the same time
        assert_eq!(
            sim.get_car_state().is_flipping,
            rs_arena.get_car_state(car_idx).is_flipping,
            "is_flipping mismatch at tick {tick}"
        );
    }
}
