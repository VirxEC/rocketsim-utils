
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

fn to_rs_state(state: &CarState) -> rs::CarState {
    rs::CarState {
        phys: rs::PhysState {
            pos: state.phys.pos,
            rot_mat: state.phys.rot_mat,
            vel: state.phys.vel,
            ang_vel: state.phys.ang_vel,
        },
        controls: rs::CarControls {
            throttle: state.controls.throttle,
            pitch: state.controls.pitch,
            yaw: state.controls.yaw,
            roll: state.controls.roll,
            jump: state.controls.jump,
            boost: state.controls.boost,
            ..rs::CarControls::DEFAULT
        },
        prev_controls: rs::CarControls {
            throttle: state.prev_controls.throttle,
            pitch: state.prev_controls.pitch,
            yaw: state.prev_controls.yaw,
            roll: state.prev_controls.roll,
            jump: state.prev_controls.jump,
            boost: state.prev_controls.boost,
            ..rs::CarControls::DEFAULT
        },
        is_on_ground: false,
        wheels_with_contact: [false; 4],
        has_jumped: state.has_jumped,
        has_double_jumped: state.has_double_jumped,
        has_flipped: state.has_flipped,
        flip_rel_torque: state.flip_rel_torque,
        flip_time: state.flip_time,
        is_flipping: state.is_flipping,
        is_jumping: state.is_jumping,
        air_time: state.air_time,
        air_time_since_jump: state.air_time_since_jump,
        boost: state.boost,
        time_since_boosted: state.time_since_boosted,
        is_boosting: state.is_boosting,
        boosting_time: state.boosting_time,
        ..rs::CarState::default()
    }
}

fn to_rs_controls(controls: CarControls) -> rs::CarControls {
    rs::CarControls {
        throttle: controls.throttle,
        pitch: controls.pitch,
        yaw: controls.yaw,
        roll: controls.roll,
        jump: controls.jump,
        boost: controls.boost,
        ..rs::CarControls::DEFAULT
    }
}

const POS_TOL: f32 = 0.5;
const VEL_TOL: f32 = 0.5;
const ANG_VEL_TOL: f32 = 0.05;
const BOOST_TOL: f32 = 1e-3;
const FLIP_TIME_TOL: f32 = 1e-4;

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

#[track_caller]
fn assert_flip_close(sim_state: &CarState, rs_state: &rs::CarState, tick: usize) {
    assert_eq!(
        sim_state.is_flipping, rs_state.is_flipping,
        "is_flipping mismatch at tick {tick}"
    );
    assert_eq!(
        sim_state.has_flipped, rs_state.has_flipped,
        "has_flipped mismatch at tick {tick}"
    );
    let flip_time_diff = (sim_state.flip_time - rs_state.flip_time).abs();
    assert!(
        flip_time_diff < FLIP_TIME_TOL,
        "flip_time diverged at tick {tick}: sim={} rs={}",
        sim_state.flip_time,
        rs_state.flip_time
    );
}

#[track_caller]
fn assert_boost_close(sim_state: &CarState, rs_state: &rs::CarState, tick: usize) {
    assert_eq!(
        sim_state.is_boosting, rs_state.is_boosting,
        "is_boosting mismatch at tick {tick}"
    );
    let boost_diff = (sim_state.boost - rs_state.boost).abs();
    assert!(
        boost_diff < BOOST_TOL,
        "boost diverged at tick {tick}: sim={} rs={}",
        sim_state.boost,
        rs_state.boost
    );
}

#[test]
fn aerial_free_flight_parity() {
    init_rs();

    let mut sim = SimArena::new(GameMode::Soccar);
    let (mut rs_arena, car_idx) = make_rs_arena();

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

        let rs_controls = to_rs_controls(controls);
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

    let mut sim_state = CarState::DEFAULT;
    sim_state.phys.pos = Vec3A::new(0.0, 0.0, 500.0);
    sim_state.phys.vel = Vec3A::new(900.0, 0.0, 0.0);
    sim_state.has_jumped = true;
    sim_state.is_jumping = false;
    sim_state.air_time_since_jump = 0.2;
    sim.set_car_state(sim_state);

    rs_arena.set_car_state(car_idx, to_rs_state(&sim_state));

    for tick in 0..120 {
        let mut controls = CarControls::DEFAULT;
        if tick == 10 {
            controls.jump = true;
            controls.yaw = 1.0;
        }

        let rs_controls = to_rs_controls(controls);
        sim.set_car_controls(controls);
        rs_arena.set_car_controls(car_idx, rs_controls);

        sim.step_tick();
        rs_arena.step_tick();

        assert_states_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);

        assert_eq!(
            sim.get_car_state().is_flipping,
            rs_arena.get_car_state(car_idx).is_flipping,
            "is_flipping mismatch at tick {tick}"
        );
    }
}

#[test]
fn aerial_boost_parity() {
    init_rs();

    let mut sim = SimArena::new(GameMode::Soccar);
    let (mut rs_arena, car_idx) = make_rs_arena();

    let mut sim_state = CarState::DEFAULT;
    sim_state.phys.pos = Vec3A::new(0.0, 0.0, 1000.0);
    sim_state.phys.vel = Vec3A::new(600.0, 0.0, 0.0);
    sim_state.boost = 100.0;
    sim.set_car_state(sim_state);

    rs_arena.set_car_state(car_idx, to_rs_state(&sim_state));

    for tick in 0..150 {
        let mut controls = CarControls::DEFAULT;
        if tick < 60 {
            controls.boost = true;
        }

        let rs_controls = to_rs_controls(controls);
        sim.set_car_controls(controls);
        rs_arena.set_car_controls(car_idx, rs_controls);

        sim.step_tick();
        rs_arena.step_tick();

        assert_states_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);
        assert_boost_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);
    }

    let expected_boost = 100.0 - 60.0 * 33.3 / 120.0;
    let sim_boost = sim.get_car_state().boost;
    assert!(
        (sim_boost - expected_boost).abs() < 0.01,
        "unexpected boost drain: sim={sim_boost} expected={expected_boost}"
    );
}

#[test]
fn aerial_forward_flip_parity() {
    init_rs();

    let mut sim = SimArena::new(GameMode::Soccar);
    let (mut rs_arena, car_idx) = make_rs_arena();

    let mut sim_state = CarState::DEFAULT;
    sim_state.phys.pos = Vec3A::new(0.0, 0.0, 1000.0);
    sim_state.phys.vel = Vec3A::new(900.0, 0.0, 100.0);
    sim_state.has_jumped = true;
    sim_state.is_jumping = false;
    sim_state.air_time_since_jump = 0.2;
    sim.set_car_state(sim_state);

    rs_arena.set_car_state(car_idx, to_rs_state(&sim_state));

    for tick in 0..150 {
        let mut controls = CarControls::DEFAULT;
        if tick == 5 {
            controls.jump = true;
            controls.pitch = -1.0;
        } else if (6..=120).contains(&tick) {
            controls.pitch = -1.0;
        }

        let rs_controls = to_rs_controls(controls);
        sim.set_car_controls(controls);
        rs_arena.set_car_controls(car_idx, rs_controls);

        sim.step_tick();
        rs_arena.step_tick();

        assert_states_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);
        assert_flip_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);
    }

    assert!(
        sim.get_car_state().has_flipped,
        "forward flip never triggered"
    );
}

#[test]
fn aerial_backflip_cancel_parity() {
    init_rs();

    let mut sim = SimArena::new(GameMode::Soccar);
    let (mut rs_arena, car_idx) = make_rs_arena();

    let mut sim_state = CarState::DEFAULT;
    sim_state.phys.pos = Vec3A::new(0.0, 0.0, 1000.0);
    sim_state.phys.vel = Vec3A::new(900.0, 0.0, 100.0);
    sim_state.has_jumped = true;
    sim_state.is_jumping = false;
    sim_state.air_time_since_jump = 0.2;
    sim.set_car_state(sim_state);

    rs_arena.set_car_state(car_idx, to_rs_state(&sim_state));

    for tick in 0..150 {
        let mut controls = CarControls::DEFAULT;
        if tick == 5 {
            controls.jump = true;
            controls.pitch = 1.0;
        } else if (6..=60).contains(&tick) {
            controls.pitch = -1.0;
        }

        let rs_controls = to_rs_controls(controls);
        sim.set_car_controls(controls);
        rs_arena.set_car_controls(car_idx, rs_controls);

        sim.step_tick();
        rs_arena.step_tick();

        assert_states_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);
        assert_flip_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);
    }

    assert!(
        sim.get_car_state().has_flipped,
        "backflip never triggered"
    );
}

#[test]
fn aerial_yaw_roll_parity() {
    init_rs();

    let mut sim = SimArena::new(GameMode::Soccar);
    let (mut rs_arena, car_idx) = make_rs_arena();

    let mut sim_state = CarState::DEFAULT;
    sim_state.phys.pos = Vec3A::new(0.0, 0.0, 1000.0);
    sim_state.phys.vel = Vec3A::new(500.0, 200.0, 0.0);
    sim.set_car_state(sim_state);

    rs_arena.set_car_state(car_idx, to_rs_state(&sim_state));

    for tick in 0..150 {
        let mut controls = CarControls::DEFAULT;
        match tick {
            0..=49 => controls.yaw = 1.0,
            50..=99 => controls.roll = 1.0,
            100..=149 => {
                controls.yaw = -0.5;
                controls.roll = 0.5;
                controls.throttle = 1.0;
            }
            _ => {}
        }

        let rs_controls = to_rs_controls(controls);
        sim.set_car_controls(controls);
        rs_arena.set_car_controls(car_idx, rs_controls);

        sim.step_tick();
        rs_arena.step_tick();

        assert_states_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);
        assert_flip_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);
    }
}

#[test]
fn aerial_double_jump_window_parity() {
    init_rs();

    let mut sim = SimArena::new(GameMode::Soccar);
    let (mut rs_arena, car_idx) = make_rs_arena();

    let mut sim_state = CarState::DEFAULT;
    sim_state.phys.pos = Vec3A::new(0.0, 0.0, 800.0);
    sim_state.phys.vel = Vec3A::new(500.0, 0.0, 0.0);
    sim_state.has_jumped = true;
    sim_state.is_jumping = false;
    sim_state.air_time_since_jump = 0.2;
    sim.set_car_state(sim_state);

    rs_arena.set_car_state(car_idx, to_rs_state(&sim_state));

    for tick in 0..120 {
        let mut controls = CarControls::DEFAULT;
        if tick == 10 {
            controls.jump = true;
        }

        let rs_controls = to_rs_controls(controls);
        sim.set_car_controls(controls);
        rs_arena.set_car_controls(car_idx, rs_controls);

        sim.step_tick();
        rs_arena.step_tick();

        assert_states_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);
        assert_flip_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);
    }

    assert!(
        sim.get_car_state().has_double_jumped,
        "double jump never triggered"
    );
    assert!(
        !sim.get_car_state().has_flipped,
        "directionless jump must not flip"
    );
}

#[test]
fn aerial_ang_vel_clamp_parity() {
    init_rs();

    let mut sim = SimArena::new(GameMode::Soccar);
    let (mut rs_arena, car_idx) = make_rs_arena();

    let mut sim_state = CarState::DEFAULT;
    sim_state.phys.pos = Vec3A::new(0.0, 0.0, 800.0);
    sim_state.phys.ang_vel = Vec3A::new(12.0, -9.0, 6.0);
    sim.set_car_state(sim_state);

    rs_arena.set_car_state(car_idx, to_rs_state(&sim_state));

    for tick in 0..120 {
        let controls = CarControls::DEFAULT;

        let rs_controls = to_rs_controls(controls);
        sim.set_car_controls(controls);
        rs_arena.set_car_controls(car_idx, rs_controls);

        sim.step_tick();
        rs_arena.step_tick();

        assert_states_close(sim.get_car_state(), rs_arena.get_car_state(car_idx), tick);
    }

    assert!(
        sim.get_car_state().phys.ang_vel.length() <= 5.5 + 1e-3,
        "angular velocity not clamped: {}",
        sim.get_car_state().phys.ang_vel
    );
}
