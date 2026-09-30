use drive_sim::{Arena as SimArena, CarControls, CarState, GameMode, PhysState};
use glam::{Mat3A, Quat, Vec3A};

fn varied_states() -> Vec<CarState> {
    let yaws = [0.0, 0.6, -1.2, 2.5, -2.9, 1.57];
    let positions = [
        Vec3A::new(0.0, -2000.0, 17.0),
        Vec3A::new(1500.0, 1000.0, 17.0),
        Vec3A::new(-2500.0, -1000.0, 17.0),
        Vec3A::new(0.0, 0.0, 100.0),
        Vec3A::new(-1800.0, 2500.0, 300.0),
        Vec3A::new(3000.0, -3000.0, 17.0),
    ];
    let vels = [
        Vec3A::ZERO,
        Vec3A::new(800.0, 0.0, 0.0),
        Vec3A::new(-500.0, 1200.0, 100.0),
        Vec3A::new(0.0, -1500.0, 0.0),
        Vec3A::new(2000.0, 500.0, -200.0),
        Vec3A::new(100.0, 100.0, 300.0),
    ];
    let ang_vels = [
        Vec3A::ZERO,
        Vec3A::new(0.0, 0.0, 2.0),
        Vec3A::new(1.0, -1.0, 0.5),
        Vec3A::new(0.0, 0.0, -3.0),
        Vec3A::new(3.0, 2.0, -1.0),
        Vec3A::ZERO,
    ];

    yaws.into_iter()
        .zip(positions)
        .zip(vels)
        .zip(ang_vels)
        .enumerate()
        .map(|(i, (((yaw, pos), vel), ang_vel))| {
            let rot_mat = Mat3A::from_quat(Quat::from_rotation_z(yaw));
            CarState {
                phys: PhysState {
                    pos,
                    rot_mat,
                    vel,
                    ang_vel,
                },
                boost: 20.0 + i as f32 * 12.0,
                controls: drive_sim::CarControls {
                    throttle: if i % 2 == 0 { 0.7 } else { -0.4 },
                    steer: (i as f32 * 0.3).sin(),
                    boost: i % 3 == 0,
                    handbrake: i % 4 == 0,
                },
                time_since_boosted: i as f32 * 0.05,
                is_boosting: i % 3 == 0,
                boosting_time: i as f32 * 0.02,
                handbrake_val: (i as f32 * 0.2).min(1.0),
                ..CarState::DEFAULT
            }
        })
        .collect()
}

fn step_case(
    arena: &mut SimArena,
    start: CarState,
    controls: CarControls,
    steps: usize,
) -> CarState {
    arena.set_car_state(start);
    for i in 0..arena.num_boost_pads() {
        arena.set_boost_pad_state(
            i,
            drive_sim::BoostPadState {
                cooldown: if i % 3 == 0 { 2.5 } else { 0.0 },
            },
        );
    }
    for _ in 0..steps {
        arena.set_car_controls(controls);
        arena.step_tick();
    }
    *arena.get_car_state()
}

#[test]
fn reused_arena_matches_fresh_after_varied_poses() {
    let starts = varied_states();
    let controls = [
        CarControls {
            throttle: 1.0,
            steer: 0.3,
            boost: true,
            handbrake: false,
        },
        CarControls {
            throttle: -0.5,
            steer: -0.8,
            boost: false,
            handbrake: true,
        },
        CarControls {
            throttle: 0.0,
            steer: 0.0,
            boost: false,
            handbrake: false,
        },
    ];

    let mut reused = SimArena::new(GameMode::Soccar, 60);
    for (case_idx, start) in starts.iter().enumerate() {
        let ctrls = controls[case_idx % controls.len()];
        let steps = 30 + case_idx * 5;

        let mut fresh = SimArena::new(GameMode::Soccar, 60);
        let expected = step_case(&mut fresh, *start, ctrls, steps);
        let got = step_case(&mut reused, *start, ctrls, steps);

        for value in [
            got.pos.x, got.pos.y, got.pos.z, got.vel.x, got.vel.y, got.vel.z,
        ] {
            assert!(
                value.is_finite(),
                "non-finite output in case {case_idx}: {got:?}"
            );
        }

        let pos_diff = (got.pos - expected.pos).length();
        let vel_diff = (got.vel - expected.vel).length();
        assert!(
            pos_diff < 1e-3 && vel_diff < 1e-3,
            "reused arena diverged in case {case_idx}: pos_diff={pos_diff} vel_diff={vel_diff}\nreused {got:?}\nfresh {expected:?}"
        );
    }
}

#[test]
fn varied_finite_inputs_stay_finite() {
    let mut arena = SimArena::new(GameMode::Soccar, 60);
    for (i, start) in varied_states().into_iter().enumerate() {
        arena.set_car_state(start);
        // History must survive teleport verbatim.
        let stored = arena.get_car_state();
        assert_eq!(
            stored.boost.to_bits(),
            start.boost.to_bits(),
            "boost lost {i}"
        );
        assert_eq!(
            stored.controls.throttle.to_bits(),
            start.controls.throttle.to_bits(),
            "throttle lost {i}"
        );
        assert_eq!(
            stored.handbrake_val.to_bits(),
            start.handbrake_val.to_bits(),
            "handbrake lost {i}"
        );
        assert_eq!(stored.is_boosting, start.is_boosting, "boosting lost {i}");
        arena.set_car_controls(CarControls {
            throttle: if i % 2 == 0 { 1.0 } else { -1.0 },
            steer: (i as f32 * 0.37).sin(),
            boost: i % 3 == 0,
            handbrake: i % 4 == 0,
        });
        for _ in 0..60 {
            arena.step_tick();
            let state = arena.get_car_state();
            assert!(
                state.pos.is_finite() && state.vel.is_finite() && state.ang_vel.is_finite(),
                "non-finite state after varied pose {i}: {state:?}"
            );
        }
    }
}
