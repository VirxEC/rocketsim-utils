use std::path::Path;

use ball_sim::{
    Arena as SimArena, BallState as SimBallState, DropshotInfo as SimDropshotInfo,
    GameMode as SimGameMode, HeatseekerInfo as SimHeatseekerInfo, PhysState as SimPhysState,
};
use ball_sim_cpp::ffi;
use glam::Vec3A;

fn mesh_path() -> &'static str {
    if Path::new("collision_meshes").exists() {
        "collision_meshes"
    } else {
        "../collision_meshes"
    }
}

static INIT: std::sync::OnceLock<()> = std::sync::OnceLock::new();

fn ensure_init() {
    INIT.get_or_init(|| {
        assert!(ball_sim_cpp::init_from_path(mesh_path(), true));
    });
    assert!(ball_sim_cpp::is_initialized());
}

fn to_vec3a(v: ffi::Vec3) -> Vec3A {
    Vec3A::new(v.x, v.y, v.z)
}

fn to_sim_state(state: ffi::BallState) -> SimBallState {
    SimBallState {
        phys: SimPhysState {
            pos: to_vec3a(state.pos),
            vel: to_vec3a(state.vel),
            ang_vel: to_vec3a(state.ang_vel),
        },
        hs_info: SimHeatseekerInfo {
            y_target_dir: state.hs_info.y_target_dir,
            cur_target_speed: state.hs_info.cur_target_speed,
            time_since_hit: state.hs_info.time_since_hit,
        },
        ds_info: SimDropshotInfo {
            charge_level: state.ds_info.charge_level,
            accumulated_hit_force: state.ds_info.accumulated_hit_force,
            y_target_dir: state.ds_info.y_target_dir,
            last_damage_tick: if state.ds_info.last_damage_tick_present {
                Some(state.ds_info.last_damage_tick)
            } else {
                None
            },
        },
        tick_count_since_kickoff: state.tick_count_since_kickoff,
    }
}

fn distinctive_state() -> ffi::BallState {
    ffi::BallState {
        pos: ffi::Vec3 {
            x: 100.0,
            y: -200.0,
            z: 300.0,
        },
        vel: ffi::Vec3 {
            x: 200.0,
            y: 100.0,
            z: 500.0,
        },
        ang_vel: ffi::Vec3 {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        },
        hs_info: ffi::HeatseekerInfo {
            y_target_dir: 1,
            cur_target_speed: 1000.0,
            time_since_hit: 0.25,
        },
        ds_info: ffi::DropshotInfo {
            charge_level: 2,
            accumulated_hit_force: 50.0,
            y_target_dir: -1,
            last_damage_tick_present: true,
            last_damage_tick: 7,
        },
        tick_count_since_kickoff: 5,
    }
}

#[track_caller]
fn assert_ffi_state_eq(a: ffi::BallState, b: ffi::BallState) {
    assert_eq!(to_vec3a(a.pos), to_vec3a(b.pos));
    assert_eq!(to_vec3a(a.vel), to_vec3a(b.vel));
    assert_eq!(to_vec3a(a.ang_vel), to_vec3a(b.ang_vel));
    assert_eq!(a.hs_info.y_target_dir, b.hs_info.y_target_dir);
    assert_eq!(a.hs_info.cur_target_speed, b.hs_info.cur_target_speed);
    assert_eq!(a.hs_info.time_since_hit, b.hs_info.time_since_hit);
    assert_eq!(a.ds_info.charge_level, b.ds_info.charge_level);
    assert_eq!(
        a.ds_info.accumulated_hit_force,
        b.ds_info.accumulated_hit_force
    );
    assert_eq!(a.ds_info.y_target_dir, b.ds_info.y_target_dir);
    assert_eq!(
        a.ds_info.last_damage_tick_present,
        b.ds_info.last_damage_tick_present
    );
    assert_eq!(a.ds_info.last_damage_tick, b.ds_info.last_damage_tick);
    assert_eq!(a.tick_count_since_kickoff, b.tick_count_since_kickoff);
}

#[test]
fn ball_state_round_trip_matches_direct_sim() {
    ensure_init();

    let default_ffi = ball_sim_cpp::ball_state_default();
    assert_ffi_state_eq(
        default_ffi,
        ffi::BallState {
            pos: ffi::Vec3 {
                x: SimBallState::DEFAULT.phys.pos.x,
                y: SimBallState::DEFAULT.phys.pos.y,
                z: SimBallState::DEFAULT.phys.pos.z,
            },
            vel: ffi::Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            ang_vel: ffi::Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            hs_info: ffi::HeatseekerInfo {
                y_target_dir: SimHeatseekerInfo::DEFAULT.y_target_dir,
                cur_target_speed: SimHeatseekerInfo::DEFAULT.cur_target_speed,
                time_since_hit: SimHeatseekerInfo::DEFAULT.time_since_hit,
            },
            ds_info: ffi::DropshotInfo {
                charge_level: SimDropshotInfo::DEFAULT.charge_level,
                accumulated_hit_force: SimDropshotInfo::DEFAULT.accumulated_hit_force,
                y_target_dir: SimDropshotInfo::DEFAULT.y_target_dir,
                last_damage_tick_present: SimDropshotInfo::DEFAULT.last_damage_tick.is_some(),
                last_damage_tick: SimDropshotInfo::DEFAULT.last_damage_tick.unwrap_or(0),
            },
            tick_count_since_kickoff: 0,
        },
    );

    let mut arena = ball_sim_cpp::arena_new(ffi::GameMode::Soccar);
    let state = distinctive_state();
    arena.set_ball_state(state);
    assert_ffi_state_eq(arena.get_ball_state(), state);

    let mut sim_arena = SimArena::new(SimGameMode::Soccar);
    sim_arena.set_ball_state(to_sim_state(state));
    let sim_state = sim_arena.get_ball_state();
    let back = arena.get_ball_state();
    assert_eq!(to_vec3a(back.pos), sim_state.phys.pos);
    assert_eq!(to_vec3a(back.vel), sim_state.phys.vel);
    assert_eq!(to_vec3a(back.ang_vel), sim_state.phys.ang_vel);
    assert_eq!(
        back.tick_count_since_kickoff,
        sim_state.tick_count_since_kickoff
    );
}

#[test]
fn soccar_stepping_matches_direct_sim() {
    ensure_init();

    let mut arena = ball_sim_cpp::arena_new(ffi::GameMode::Soccar);
    let mut sim_arena = SimArena::new(SimGameMode::Soccar);

    let state = distinctive_state();
    arena.set_ball_state(state);
    sim_arena.set_ball_state(to_sim_state(state));

    for _ in 0..120 {
        arena.step_tick();
        _ = sim_arena.step_tick();

        let back = arena.get_ball_state();
        let sim_state = sim_arena.get_ball_state();

        let pos_diff = (to_vec3a(back.pos) - sim_state.phys.pos).length();
        let vel_diff = (to_vec3a(back.vel) - sim_state.phys.vel).length();
        let ang_diff = (to_vec3a(back.ang_vel) - sim_state.phys.ang_vel).length();
        assert!(
            pos_diff == 0.0 && vel_diff == 0.0 && ang_diff == 0.0,
            "states diverged: pos_diff={pos_diff}, vel_diff={vel_diff}, ang_diff={ang_diff}"
        );
        assert_eq!(
            back.tick_count_since_kickoff,
            sim_state.tick_count_since_kickoff
        );
    }
}

#[test]
fn arena_new_covers_supported_game_modes() {
    ensure_init();

    for mode in [
        ffi::GameMode::Soccar,
        ffi::GameMode::Heatseeker,
        ffi::GameMode::TheVoid,
    ] {
        let mut arena = ball_sim_cpp::arena_new(mode);
        let state = ball_sim_cpp::ball_state_default();
        arena.set_ball_state(state);
        assert_ffi_state_eq(arena.get_ball_state(), state);
    }
}
