//! Parity tests against the full version of RocketSim v3.
//!
//! These run driving scenarios in both this crate's `Arena` and RocketSim's
//! `Arena` side-by-side, asserting that the car states stay in sync.

use std::path::Path;

use drive_sim::{Arena as SimArena, CarControls, CarState, GameMode};
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
        // A single far-away boost pad so both sims only exercise driving
        // along the flat part of the field without pad pickups
        custom_boost_pads: Some(vec![rs::BoostPadConfig {
            pos: Vec3A::new(4500.0, 4500.0, 0.0),
            is_big: false,
        }]),
        ..Default::default()
    });
    let car_idx = arena.add_car(rs::Team::Blue, rs::CarBodyConfig::OCTANE);
    (arena, car_idx)
}

const POS_TOL: f32 = 1.0; // uu
const VEL_TOL: f32 = 1.0; // uu/s

#[track_caller]
fn assert_states_close(sim_state: &CarState, rs_state: &rs::CarState, tick: usize) {
    let pos_diff = (sim_state.phys.pos - rs_state.phys.pos).length();
    let vel_diff = (sim_state.phys.vel - rs_state.phys.vel).length();
    assert!(
        pos_diff < POS_TOL && vel_diff < VEL_TOL,
        "states diverged at tick {tick}: pos_diff={pos_diff}, vel_diff={vel_diff}\n\
         sim:  pos={} vel={}\n\
         rs:   pos={} vel={}",
        sim_state.phys.pos,
        sim_state.phys.vel,
        rs_state.phys.pos,
        rs_state.phys.vel
    );
}

#[test]
fn straight_line_drive_parity() {
    init_rs();

    let mut sim = SimArena::new(GameMode::Soccar, 120);
    let (mut rs_arena, car_idx) = make_rs_arena();

    // Spawn both cars on the flat part of the field, away from the ball and
    // any curved surfaces, facing positive-x
    let start_pos = Vec3A::new(0.0, -2000.0, 17.0);

    let mut sim_state = CarState::DEFAULT;
    sim_state.phys.pos = start_pos;
    sim.set_car_state(sim_state);

    let mut rs_state = rs::CarState {
        boost: sim_state.boost,
        ..rs::CarState::default()
    };
    rs_state.phys.pos = start_pos;
    rs_arena.set_car_state(car_idx, rs_state);

    for tick in 0..300 {
        sim.set_car_controls(CarControls {
            throttle: 1.0,
            ..CarControls::DEFAULT
        });
        rs_arena.set_car_controls(
            car_idx,
            rs::CarControls {
                throttle: 1.0,
                ..rs::CarControls::DEFAULT
            },
        );

        sim.step_tick();
        rs_arena.step_tick();

        assert_states_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);
    }
}

#[test]
fn turning_drive_parity() {
    init_rs();

    let mut sim = SimArena::new(GameMode::Soccar, 120);
    let (mut rs_arena, car_idx) = make_rs_arena();

    let start_pos = Vec3A::new(0.0, -2000.0, 17.0);

    let mut sim_state = CarState::DEFAULT;
    sim_state.phys.pos = start_pos;
    sim.set_car_state(sim_state);

    let mut rs_state = rs::CarState {
        boost: sim_state.boost,
        ..rs::CarState::default()
    };
    rs_state.phys.pos = start_pos;
    rs_arena.set_car_state(car_idx, rs_state);

    for tick in 0..240 {
        // Throttle + steer + occasional handbrake for a powerslide-ish arc
        let mut controls = CarControls {
            throttle: 1.0,
            ..CarControls::DEFAULT
        };
        controls.steer = 0.6;
        controls.handbrake = (30..60).contains(&tick) || (120..150).contains(&tick);

        sim.set_car_controls(controls);
        rs_arena.set_car_controls(
            car_idx,
            rs::CarControls {
                throttle: controls.throttle,
                steer: controls.steer,
                handbrake: controls.handbrake,
                ..rs::CarControls::DEFAULT
            },
        );

        sim.step_tick();
        rs_arena.step_tick();

        assert_states_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);
    }
}
