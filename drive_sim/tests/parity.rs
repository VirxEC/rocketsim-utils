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
        custom_boost_pads: Some(vec![rs::BoostPadConfig {
            pos: Vec3A::new(4500.0, 4500.0, 0.0),
            is_big: false,
        }]),
        ..Default::default()
    });
    let car_idx = arena.add_car(rs::Team::Blue, rs::CarBodyConfig::OCTANE);
    (arena, car_idx)
}

const POS_TOL: f32 = 1.0;
const VEL_TOL: f32 = 1.0;

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

const BOOST_TOL: f32 = 0.6;

#[track_caller]
fn assert_boost_close(sim_state: &CarState, rs_state: &rs::CarState, tick: usize) {
    let boost_diff = (sim_state.boost - rs_state.boost).abs();
    assert!(
        boost_diff < BOOST_TOL,
        "boost diverged at tick {tick}: sim={} rs={}",
        sim_state.boost,
        rs_state.boost
    );
}

fn set_both_states(
    sim: &mut SimArena,
    rs_arena: &mut rs::Arena,
    car_idx: usize,
    pos: Vec3A,
    boost: f32,
) {
    let mut sim_state = CarState::DEFAULT;
    sim_state.phys.pos = pos;
    sim_state.boost = boost;
    sim.set_car_state(sim_state);

    let mut rs_state = rs::CarState {
        boost,
        ..rs::CarState::default()
    };
    rs_state.phys.pos = pos;
    rs_arena.set_car_state(car_idx, rs_state);
}

fn set_both_controls(
    sim: &mut SimArena,
    rs_arena: &mut rs::Arena,
    car_idx: usize,
    controls: CarControls,
) {
    sim.set_car_controls(controls);
    rs_arena.set_car_controls(
        car_idx,
        rs::CarControls {
            throttle: controls.throttle,
            steer: controls.steer,
            boost: controls.boost,
            handbrake: controls.handbrake,
            ..rs::CarControls::DEFAULT
        },
    );
}

#[test]
fn straight_line_drive_parity() {
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

#[test]
fn boost_drive_parity() {
    init_rs();

    let mut sim = SimArena::new(GameMode::Soccar, 120);
    let (mut rs_arena, car_idx) = make_rs_arena();

    set_both_states(&mut sim, &mut rs_arena, car_idx, Vec3A::new(0.0, -2000.0, 17.0), 40.0);

    for tick in 0..240 {
        let controls = if tick < 120 {
            CarControls {
                throttle: 1.0,
                boost: true,
                ..CarControls::DEFAULT
            }
        } else if tick < 200 {
            CarControls {
                throttle: 0.0,
                boost: true,
                ..CarControls::DEFAULT
            }
        } else {
            CarControls::DEFAULT
        };

        set_both_controls(&mut sim, &mut rs_arena, car_idx, controls);

        sim.step_tick();
        rs_arena.step_tick();

        assert_states_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);
        assert_boost_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);
    }
}

#[test]
fn boost_pickup_parity() {
    init_rs();

    let mut sim = SimArena::new(GameMode::Soccar, 120);
    let mut rs_arena = rs::Arena::new_with_config(rs::ArenaConfig {
        game_mode: rs::GameMode::Soccar,
        ..Default::default()
    });
    let car_idx = rs_arena.add_car(rs::Team::Blue, rs::CarBodyConfig::OCTANE);
    assert_eq!(sim.num_boost_pads(), rs_arena.num_boost_pads());

    set_both_states(&mut sim, &mut rs_arena, car_idx, Vec3A::new(0.0, -1024.0, 17.0), 0.0);

    for tick in 0..8 {
        set_both_controls(&mut sim, &mut rs_arena, car_idx, CarControls::DEFAULT);

        sim.step_tick();
        rs_arena.step_tick();

        assert_states_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);
        assert_boost_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);
    }

    assert!(
        (sim.get_car_state().boost - 12.0).abs() < BOOST_TOL,
        "sim boost after pickup: {}",
        sim.get_car_state().boost
    );
    assert!(
        (rs_arena.get_car_state(car_idx).boost - 12.0).abs() < BOOST_TOL,
        "rs boost after pickup: {}",
        rs_arena.get_car_state(car_idx).boost
    );

    for tick in 8..208 {
        set_both_controls(
            &mut sim,
            &mut rs_arena,
            car_idx,
            CarControls {
                throttle: 1.0,
                boost: true,
                ..CarControls::DEFAULT
            },
        );

        sim.step_tick();
        rs_arena.step_tick();

        assert_states_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);
        assert_boost_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);
    }
}

#[test]
fn brake_coast_parity() {
    init_rs();

    let mut sim = SimArena::new(GameMode::Soccar, 120);
    let (mut rs_arena, car_idx) = make_rs_arena();

    set_both_states(&mut sim, &mut rs_arena, car_idx, Vec3A::new(0.0, -2000.0, 17.0), 33.0);

    for tick in 0..360 {
        let controls = if tick < 120 {
            CarControls {
                throttle: 1.0,
                ..CarControls::DEFAULT
            }
        } else if tick < 240 {
            CarControls::DEFAULT
        } else {
            CarControls {
                throttle: -1.0,
                ..CarControls::DEFAULT
            }
        };

        set_both_controls(&mut sim, &mut rs_arena, car_idx, controls);

        sim.step_tick();
        rs_arena.step_tick();

        assert_states_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);
    }
}

#[test]
fn handbrake_circle_parity() {
    init_rs();

    let mut sim = SimArena::new(GameMode::Soccar, 120);
    let (mut rs_arena, car_idx) = make_rs_arena();

    set_both_states(&mut sim, &mut rs_arena, car_idx, Vec3A::new(0.0, -2000.0, 17.0), 33.0);

    for tick in 0..150 {
        set_both_controls(
            &mut sim,
            &mut rs_arena,
            car_idx,
            CarControls {
                throttle: 1.0,
                steer: 1.0,
                handbrake: true,
                ..CarControls::DEFAULT
            },
        );

        sim.step_tick();
        rs_arena.step_tick();

        assert_states_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);
    }
}

#[test]
fn spawn_sticky_gate_parity() {
    init_rs();

    let mut sim = SimArena::new(GameMode::Soccar, 120);
    let (mut rs_arena, car_idx) = make_rs_arena();

    set_both_states(&mut sim, &mut rs_arena, car_idx, Vec3A::new(0.0, -2000.0, 17.0), 33.0);

    for tick in 0..200 {
        set_both_controls(
            &mut sim,
            &mut rs_arena,
            car_idx,
            CarControls {
                throttle: 1.0,
                ..CarControls::DEFAULT
            },
        );

        sim.step_tick();
        rs_arena.step_tick();

        let sim_state = sim.get_car_state();
        let rs_state = rs_arena.get_car_state(car_idx);
        let pos_diff = (sim_state.phys.pos - rs_state.phys.pos).length();
        let vel_diff = (sim_state.phys.vel - rs_state.phys.vel).length();
        assert!(
            pos_diff < 0.5 && vel_diff < 0.5,
            "states diverged at tick {tick}: pos_diff={pos_diff}, vel_diff={vel_diff}"
        );
    }
}
