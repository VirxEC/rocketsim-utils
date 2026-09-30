use std::{path::PathBuf, sync::OnceLock};

use ball_sim::{Arena as SimArena, BallState, GameMode, PhysState, init};
use glam::Vec3A;

fn ensure_init() {
    static INIT: OnceLock<()> = OnceLock::new();
    INIT.get_or_init(|| {
        let dir: PathBuf = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../collision_meshes");
        init(&dir, true).unwrap();
    });
}

fn varied_states() -> Vec<BallState> {
    let positions = [
        Vec3A::new(0.0, 0.0, 100.0),
        Vec3A::new(2000.0, -2000.0, 200.0),
        Vec3A::new(-3000.0, 1000.0, 500.0),
        Vec3A::new(0.0, 5000.0, 300.0),
        Vec3A::new(4000.0, 0.0, 100.0),
        Vec3A::new(-1000.0, -4000.0, 800.0),
    ];
    let vels = [
        Vec3A::ZERO,
        Vec3A::new(1000.0, 500.0, 200.0),
        Vec3A::new(-800.0, -1200.0, 400.0),
        Vec3A::new(0.0, 2000.0, -300.0),
        Vec3A::new(-1500.0, 800.0, 100.0),
        Vec3A::new(300.0, -300.0, 900.0),
    ];

    positions
        .into_iter()
        .zip(vels)
        .enumerate()
        .map(|(i, (pos, vel))| {
            let mut state = BallState {
                phys: PhysState {
                    pos,
                    vel,
                    ang_vel: Vec3A::ZERO,
                },
                ..Default::default()
            };
            // Vary history. Teleport must preserve it verbatim.
            state.hs_info.y_target_dir = if i % 2 == 0 { 1 } else { -1 };
            state.hs_info.cur_target_speed = 2000.0 + i as f32 * 200.0;
            state.hs_info.time_since_hit = i as f32 * 0.1;
            state.ds_info.charge_level = 1 + (i % 3) as u8;
            state.ds_info.accumulated_hit_force = i as f32 * 10.0;
            state.ds_info.y_target_dir = if i % 2 == 0 { 1 } else { -1 };
            state.tick_count_since_kickoff = 100 + i as u64 * 10;
            state
        })
        .collect()
}

fn step_case(arena: &mut SimArena, start: BallState, steps: usize) -> PhysState {
    arena.set_ball_state(start);
    for _ in 0..steps {
        arena.step_tick();
    }
    arena.get_ball_state().phys
}

#[test]
fn reused_arena_matches_fresh_after_varied_poses() {
    // TheVoid needs no collision meshes.
    let starts = varied_states();
    let mut reused = SimArena::new(GameMode::TheVoid);
    for (case_idx, start) in starts.iter().enumerate() {
        let steps = 30 + case_idx * 5;

        let mut fresh = SimArena::new(GameMode::TheVoid);
        let expected = step_case(&mut fresh, *start, steps);
        let got = step_case(&mut reused, *start, steps);

        assert!(
            got.pos.is_finite() && got.vel.is_finite(),
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
fn teleport_preserves_history() {
    let mut arena = SimArena::new(GameMode::TheVoid);
    for (i, start) in varied_states().into_iter().enumerate() {
        arena.set_ball_state(start);
        let stored = arena.get_ball_state();
        assert_eq!(
            stored.hs_info.y_target_dir, start.hs_info.y_target_dir,
            "hs dir lost {i}"
        );
        assert_eq!(
            stored.hs_info.cur_target_speed.to_bits(),
            start.hs_info.cur_target_speed.to_bits(),
            "hs speed lost {i}"
        );
        assert_eq!(
            stored.ds_info.charge_level, start.ds_info.charge_level,
            "ds level lost {i}"
        );
        assert_eq!(
            stored.tick_count_since_kickoff, start.tick_count_since_kickoff,
            "kickoff tick lost {i}"
        );
    }
}

#[test]
fn soccar_reused_matches_fresh_after_contact_then_teleport() {
    ensure_init();

    // Warm with floor contact. Then teleport to a different finite pose.
    // Stale manifolds must not leak. Reused must match fresh.
    let warm = BallState {
        phys: PhysState {
            pos: Vec3A::new(0.0, 0.0, 93.15),
            vel: Vec3A::ZERO,
            ang_vel: Vec3A::ZERO,
        },
        ..Default::default()
    };
    let targets = [
        BallState {
            phys: PhysState {
                pos: Vec3A::new(800.0, -1000.0, 500.0),
                vel: Vec3A::new(500.0, 300.0, -200.0),
                ang_vel: Vec3A::ZERO,
            },
            ..Default::default()
        },
        BallState {
            phys: PhysState {
                pos: Vec3A::new(3900.0, 0.0, 300.0),
                vel: Vec3A::new(1500.0, 0.0, 0.0),
                ang_vel: Vec3A::ZERO,
            },
            ..Default::default()
        },
        BallState {
            phys: PhysState {
                pos: Vec3A::new(0.0, 4900.0, 400.0),
                vel: Vec3A::new(0.0, 1200.0, 300.0),
                ang_vel: Vec3A::ZERO,
            },
            ..Default::default()
        },
    ];

    let mut reused = SimArena::new(GameMode::Soccar);
    for (case_idx, target) in targets.iter().enumerate() {
        // Warm the reused arena with real contact.
        reused.set_ball_state(warm);
        for _ in 0..30 {
            reused.step_tick();
        }
        assert!(
            reused.num_persistent_manifolds() > 0,
            "warmup created no manifold {case_idx}"
        );
        // Teleport to the target. This must clear stale contacts.
        reused.set_ball_state(*target);
        assert_eq!(
            reused.num_persistent_manifolds(),
            0,
            "teleport kept stale manifolds {case_idx}"
        );
        for _ in 0..60 {
            reused.step_tick();
        }
        let got = reused.get_ball_state().phys;

        let mut fresh = SimArena::new(GameMode::Soccar);
        assert_eq!(fresh.num_persistent_manifolds(), 0);
        fresh.set_ball_state(*target);
        for _ in 0..60 {
            fresh.step_tick();
        }
        let expected = fresh.get_ball_state().phys;

        assert!(
            got.pos.is_finite() && got.vel.is_finite(),
            "non-finite Soccar output {case_idx}: {got:?}"
        );
        let pos_diff = (got.pos - expected.pos).length();
        let vel_diff = (got.vel - expected.vel).length();
        assert!(
            pos_diff < 1e-3 && vel_diff < 1e-3,
            "Soccar reused diverged {case_idx}: pos_diff={pos_diff} vel_diff={vel_diff}"
        );
    }
}
