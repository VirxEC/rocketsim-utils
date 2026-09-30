use air_sim::{Arena as SimArena, CarControls, CarState, GameMode, PhysState};
use glam::{Mat3A, Quat, Vec3A};

fn varied_states() -> Vec<CarState> {
    let rotations = [
        Quat::IDENTITY,
        Quat::from_rotation_z(1.2),
        Quat::from_rotation_x(0.5),
        Quat::from_rotation_y(-0.7),
        Quat::from_euler(glam::EulerRot::ZYX, 2.0, 0.4, -0.3),
        Quat::from_rotation_z(-2.4),
    ];
    let positions = [
        Vec3A::new(0.0, 0.0, 500.0),
        Vec3A::new(1000.0, -1500.0, 800.0),
        Vec3A::new(-2000.0, 1000.0, 1200.0),
        Vec3A::new(500.0, 2500.0, 300.0),
        Vec3A::new(-1000.0, -2000.0, 1500.0),
        Vec3A::new(0.0, 0.0, 100.0),
    ];
    let vels = [
        Vec3A::new(400.0, 0.0, 0.0),
        Vec3A::new(-600.0, 800.0, 200.0),
        Vec3A::new(0.0, 0.0, -500.0),
        Vec3A::new(1200.0, -400.0, 600.0),
        Vec3A::ZERO,
        Vec3A::new(200.0, 200.0, 100.0),
    ];

    rotations
        .into_iter()
        .zip(positions)
        .zip(vels)
        .enumerate()
        .map(|(i, ((rot, pos), vel))| {
            let rot_mat = Mat3A::from_quat(rot);
            let mut state = CarState::DEFAULT;
            state.phys = PhysState {
                pos,
                rot_mat,
                rot_quat: rot,
                vel,
                ang_vel: Vec3A::new(0.5 * i as f32, -0.3 * i as f32, 0.2 * i as f32),
            };
            state.boost = 30.0 + i as f32 * 10.0;
            // Vary history to cover jump and flip residues.
            state.has_jumped = i % 2 == 0;
            state.has_double_jumped = i % 3 == 0;
            state.has_flipped = i % 4 == 0;
            state.is_flipping = false;
            state.is_jumping = false;
            state.air_time = i as f32 * 0.1;
            state.air_time_since_jump = i as f32 * 0.05;
            state.prev_controls = CarControls::DEFAULT;
            state
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
            pitch: 0.2,
            yaw: -0.1,
            roll: 0.0,
            jump: false,
            boost: true,
        },
        CarControls {
            throttle: 0.0,
            pitch: -0.5,
            yaw: 0.4,
            roll: 0.1,
            jump: true,
            boost: false,
        },
        CarControls::DEFAULT,
    ];

    let mut reused = SimArena::new(GameMode::Soccar);
    for (case_idx, start) in starts.iter().enumerate() {
        let ctrls = controls[case_idx % controls.len()];
        let steps = 30 + case_idx * 7;

        let mut fresh = SimArena::new(GameMode::Soccar);
        let expected = step_case(&mut fresh, *start, ctrls, steps);
        let got = step_case(&mut reused, *start, ctrls, steps);

        assert!(
            got.pos.is_finite() && got.vel.is_finite() && got.ang_vel.is_finite(),
            "non-finite output in case {case_idx}: {got:?}"
        );

        let pos_diff = (got.pos - expected.pos).length();
        let vel_diff = (got.vel - expected.vel).length();
        assert!(
            pos_diff < 1e-3 && vel_diff < 1e-3,
            "reused arena diverged in case {case_idx}: pos_diff={pos_diff} vel_diff={vel_diff}"
        );
    }
}

#[test]
fn varied_finite_inputs_stay_finite_and_preserve_history() {
    let mut arena = SimArena::new(GameMode::Soccar);
    for (i, start) in varied_states().into_iter().enumerate() {
        arena.set_car_state(start);
        // History must survive the teleport verbatim.
        let stored = arena.get_car_state();
        assert_eq!(
            stored.has_jumped, start.has_jumped,
            "history lost in case {i}"
        );
        assert_eq!(
            stored.has_double_jumped, start.has_double_jumped,
            "history lost in case {i}"
        );
        assert_eq!(
            stored.has_flipped, start.has_flipped,
            "history lost in case {i}"
        );

        arena.set_car_controls(CarControls {
            throttle: 1.0,
            boost: i % 2 == 0,
            ..CarControls::DEFAULT
        });
        for _ in 0..45 {
            arena.step_tick();
            let state = arena.get_car_state();
            assert!(
                state.pos.is_finite() && state.vel.is_finite() && state.ang_vel.is_finite(),
                "non-finite state after varied pose {i}: {state:?}"
            );
        }
    }
}
