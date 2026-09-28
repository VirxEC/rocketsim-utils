use std::path::Path;

use glam::{EulerRot, Mat3A, Quat, Vec3A};
use rand::{RngExt, SeedableRng, rngs::SmallRng};
use rocketsim as rs;
use turn_sim::{Car, car_controls::CarControls, consts::car};

const DT: f32 = 1.0 / 120.0;
const SPAWN_Z: f32 = 1500.0;

const ROT_TOL: f32 = 0.005;
const ANG_VEL_TOL: f32 = 0.01;
const SAT_OVERSHOOT_TOL: f32 = 0.15;

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

fn initial_states(rot: Quat, ang_vel: Vec3A) -> (Car, rs::CarState) {
    let sim_car = Car { rot, ang_vel };
    let rs_state = rs::CarState {
        phys: rs::PhysState {
            pos: Vec3A::new(0.0, 0.0, SPAWN_Z),
            rot_mat: Mat3A::from_quat(rot),
            vel: Vec3A::ZERO,
            ang_vel,
        },
        is_on_ground: false,
        wheels_with_contact: [None; 4],
        ..rs::CarState::default()
    };
    (sim_car, rs_state)
}

fn to_rs_controls(controls: CarControls) -> rs::CarControls {
    rs::CarControls {
        pitch: controls.pitch,
        yaw: controls.yaw,
        roll: controls.roll,
        ..rs::CarControls::DEFAULT
    }
}

fn clamp_ang_vel(v: Vec3A) -> Vec3A {
    if v.length_squared() > car::MAX_ANG_SPEED * car::MAX_ANG_SPEED {
        v.normalize() * car::MAX_ANG_SPEED
    } else {
        v
    }
}

fn rot_angle_diff(a: Quat, b: Quat) -> f32 {
    2.0 * a.dot(b).abs().clamp(-1.0, 1.0).acos()
}

#[track_caller]
fn assert_turn_close(sim_car: &Car, rs_state: &rs::CarState, tick: usize) {
    let rs_quat = Quat::from_mat3a(&rs_state.phys.rot_mat);
    let rot_diff = rot_angle_diff(sim_car.rot, rs_quat);
    let ang_vel_diff = (sim_car.ang_vel - clamp_ang_vel(rs_state.phys.ang_vel)).length();
    assert!(
        rot_diff < ROT_TOL && ang_vel_diff < ANG_VEL_TOL,
        "turn states diverged at tick {tick}: rot_diff={rot_diff} ang_vel_diff={ang_vel_diff}\n\
         sim: rot={} ang={}\n\
         rs:  rot={} ang={}",
        sim_car.rot_mat(),
        sim_car.ang_vel,
        rs_state.phys.rot_mat,
        rs_state.phys.ang_vel,
    );
}

fn run_scenario(
    rot: Quat,
    ang_vel: Vec3A,
    ticks: usize,
    schedule: impl Fn(usize) -> CarControls,
) -> (Car, rs::CarState) {
    init_rs();
    let (mut sim_car, rs_state) = initial_states(rot, ang_vel);
    let (mut rs_arena, car_idx) = make_rs_arena();
    rs_arena.set_car_state(car_idx, rs_state);

    for tick in 0..ticks {
        let controls = schedule(tick);
        sim_car.step_turn(controls, DT);
        rs_arena.set_car_controls(car_idx, to_rs_controls(controls));
        rs_arena.step_tick();

        let rs_state = *rs_arena.get_car_state(car_idx);
        assert_turn_close(&sim_car, &rs_state, tick);

        assert!(
            !rs_state.is_on_ground && rs_state.wheels_with_contact == [None; 4],
            "car left suspended flight at tick {tick}"
        );
    }

    (sim_car, *rs_arena.get_car_state(car_idx))
}

#[test]
fn yaw_only_parity() {
    run_scenario(Quat::IDENTITY, Vec3A::ZERO, 120, |_| {
        CarControls::DEFAULT.with_yaw(1.0)
    });
}

#[test]
fn pitch_only_parity() {
    run_scenario(Quat::IDENTITY, Vec3A::ZERO, 120, |_| {
        CarControls::DEFAULT.with_pitch(-1.0)
    });
}

#[test]
fn roll_spin_damping_parity() {
    run_scenario(Quat::IDENTITY, Vec3A::ZERO, 150, |tick| {
        if tick < 60 {
            CarControls::DEFAULT.with_roll(1.0)
        } else {
            CarControls::DEFAULT
        }
    });
}

#[test]
fn combined_inputs_parity() {
    let rot = Quat::from_euler(EulerRot::ZYX, 0.7, -0.4, 1.9);
    let (sim_car, rs_state) =
        run_scenario(rot, Vec3A::new(0.5, -0.3, 0.2), 180, |tick| match tick {
            0..60 => CarControls {
                pitch: 0.5,
                yaw: -1.0,
                roll: 0.25,
            },
            60..120 => CarControls::DEFAULT,
            _ => CarControls {
                pitch: -0.75,
                yaw: 0.5,
                roll: -1.0,
            },
        });
    assert!(
        rot_angle_diff(sim_car.rot, rot) > 1.0,
        "combined scenario barely rotated the car"
    );
    let _ = rs_state;
}

#[test]
fn ang_speed_clamp_parity() {
    let over = Vec3A::new(12.0, -9.0, 6.0);
    assert!(over.length() > car::MAX_ANG_SPEED);
    let (sim_car, _) = run_scenario(Quat::IDENTITY, clamp_ang_vel(over), 120, |_| {
        CarControls::DEFAULT
    });
    assert!(
        sim_car.ang_vel.length() <= car::MAX_ANG_SPEED + 1e-3,
        "angular velocity not clamped: {}",
        sim_car.ang_vel
    );
}

#[test]
fn saturated_spin_parity() {
    let (_, rs_state) = run_scenario(Quat::IDENTITY, Vec3A::ZERO, 150, |_| {
        CarControls::DEFAULT.with_roll(1.0)
    });
    let rs_speed = rs_state.phys.ang_vel.length();
    assert!(
        rs_speed <= car::MAX_ANG_SPEED + SAT_OVERSHOOT_TOL,
        "saturated spin overshoot out of bounds: {rs_speed}"
    );
    assert!(
        rs_speed >= car::MAX_ANG_SPEED - ANG_VEL_TOL,
        "spin never saturated: {rs_speed}"
    );
}

#[test]
fn randomized_turn_parity() {
    let mut rng = SmallRng::seed_from_u64(7);
    for _ in 0..8 {
        let rot = Quat::from_euler(
            EulerRot::ZYX,
            rng.random_range(-std::f32::consts::PI..=std::f32::consts::PI),
            rng.random_range(-std::f32::consts::PI..=std::f32::consts::PI),
            rng.random_range(-std::f32::consts::PI..=std::f32::consts::PI),
        );
        let ang_vel = Vec3A::new(
            rng.random_range(-3.0..=3.0),
            rng.random_range(-3.0..=3.0),
            rng.random_range(-3.0..=3.0),
        );
        let segments: Vec<CarControls> = (0..14)
            .map(|_| CarControls {
                pitch: rng.random_range(-1.0..=1.0),
                yaw: rng.random_range(-1.0..=1.0),
                roll: rng.random_range(-1.0..=1.0),
            })
            .collect();
        run_scenario(rot, ang_vel, 200, |tick| {
            segments[(tick / 15).min(segments.len() - 1)]
        });
    }
}
