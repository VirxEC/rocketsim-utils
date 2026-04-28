use ahash::AHashMap;
use ball_sim::{
    Arena as SimArena, BallState as SimBallState, DropshotInfo as SimDropshotInfo,
    GameMode as SimGameMode, HeatseekerInfo as SimHeatseekerInfo,
};

fn to_vec3a(v: ffi::Vec3) -> glam::Vec3A {
    glam::Vec3A::new(v.x, v.y, v.z)
}

fn from_vec3a(v: glam::Vec3A) -> ffi::Vec3 {
    ffi::Vec3 {
        x: v.x,
        y: v.y,
        z: v.z,
    }
}

fn to_hs_info(info: ffi::HeatseekerInfo) -> SimHeatseekerInfo {
    SimHeatseekerInfo {
        y_target_dir: info.y_target_dir,
        cur_target_speed: info.cur_target_speed,
        time_since_hit: info.time_since_hit,
    }
}

fn from_hs_info(info: SimHeatseekerInfo) -> ffi::HeatseekerInfo {
    ffi::HeatseekerInfo {
        y_target_dir: info.y_target_dir,
        cur_target_speed: info.cur_target_speed,
        time_since_hit: info.time_since_hit,
    }
}

fn to_ds_info(info: ffi::DropshotInfo) -> SimDropshotInfo {
    SimDropshotInfo {
        charge_level: info.charge_level,
        accumulated_hit_force: info.accumulated_hit_force,
        y_target_dir: info.y_target_dir,
        has_damaged: info.has_damaged,
        last_damage_tick: info.last_damage_tick,
    }
}

fn from_ds_info(info: SimDropshotInfo) -> ffi::DropshotInfo {
    ffi::DropshotInfo {
        charge_level: info.charge_level,
        accumulated_hit_force: info.accumulated_hit_force,
        y_target_dir: info.y_target_dir,
        has_damaged: info.has_damaged,
        last_damage_tick: info.last_damage_tick,
    }
}

fn to_game_mode(game_mode: ffi::GameMode) -> SimGameMode {
    match game_mode {
        ffi::GameMode::Hoops => SimGameMode::Hoops,
        ffi::GameMode::Heatseeker => SimGameMode::Heatseeker,
        ffi::GameMode::Snowday => SimGameMode::Snowday,
        ffi::GameMode::Dropshot => SimGameMode::Dropshot,
        ffi::GameMode::TheVoid => SimGameMode::TheVoid,
        _ => SimGameMode::Soccar,
    }
}

#[cxx::bridge]
mod ffi {
    #[derive(Clone, Copy, Debug)]
    pub struct Vec3 {
        pub x: f32,
        pub y: f32,
        pub z: f32,
    }

    #[derive(Clone, Copy, Debug)]
    pub struct HeatseekerInfo {
        pub y_target_dir: i8,
        pub cur_target_speed: f32,
        pub time_since_hit: f32,
    }

    #[derive(Clone, Copy, Debug)]
    pub struct DropshotInfo {
        pub charge_level: i32,
        pub accumulated_hit_force: f32,
        pub y_target_dir: i8,
        pub has_damaged: bool,
        pub last_damage_tick: u64,
    }

    #[derive(Clone, Copy, Debug)]
    pub struct BallState {
        pub pos: Vec3,
        pub vel: Vec3,
        pub ang_vel: Vec3,
        pub hs_info: HeatseekerInfo,
        pub ds_info: DropshotInfo,
        pub last_extra_hit_tick_present: bool,
        pub last_extra_hit_tick: u64,
        pub tick_count_since_kickoff: u64,
    }

    #[derive(Clone, Debug)]
    pub struct MeshFile {
        pub game_mode: GameMode,
        pub data: Vec<u8>,
    }

    #[derive(Clone, Copy, Debug)]
    pub enum GameMode {
        Soccar,
        Hoops,
        Heatseeker,
        Snowday,
        Dropshot,
        TheVoid,
    }

    extern "Rust" {
        type Arena;

        fn init_from_default(silent: bool) -> bool;
        fn init_from_path(path: &str, silent: bool) -> bool;
        fn init_from_mem(mesh_files: Vec<MeshFile>, silent: bool) -> bool;
        fn is_initialized() -> bool;

        fn ball_state_default() -> BallState;

        fn arena_new(game_mode: GameMode) -> Box<Arena>;
        fn set_ball_state(self: &mut Arena, state: BallState);
        fn get_ball_state(self: &Arena) -> BallState;
        fn step_tick(self: &mut Arena);
    }
}

pub struct Arena {
    inner: SimArena,
}

#[must_use]
pub fn init_from_default(silent: bool) -> bool {
    ball_sim::init_from_default(silent).is_ok()
}

#[must_use]
pub fn init_from_path(path: &str, silent: bool) -> bool {
    ball_sim::init(path, silent).is_ok()
}

#[must_use]
pub fn init_from_mem(mesh_files: Vec<ffi::MeshFile>, silent: bool) -> bool {
    let mut mesh_map: AHashMap<SimGameMode, Vec<Vec<u8>>> = AHashMap::new();
    for mesh in mesh_files {
        mesh_map
            .entry(to_game_mode(mesh.game_mode))
            .or_default()
            .push(mesh.data);
    }
    ball_sim::init_from_mem(mesh_map, silent).is_ok()
}

#[must_use]
pub fn is_initialized() -> bool {
    ball_sim::is_initialized()
}

#[must_use]
pub fn ball_state_default() -> ffi::BallState {
    let state = SimBallState::DEFAULT;
    ffi::BallState {
        pos: from_vec3a(state.phys.pos),
        vel: from_vec3a(state.phys.vel),
        ang_vel: from_vec3a(state.phys.ang_vel),
        hs_info: from_hs_info(state.hs_info),
        ds_info: from_ds_info(state.ds_info),
        last_extra_hit_tick_present: state.last_extra_hit_tick.is_some(),
        last_extra_hit_tick: state.last_extra_hit_tick.unwrap_or(0),
        tick_count_since_kickoff: state.tick_count_since_kickoff,
    }
}

#[must_use]
pub fn arena_new(game_mode: ffi::GameMode) -> Box<Arena> {
    let gm = to_game_mode(game_mode);

    Box::new(Arena {
        inner: SimArena::new(gm),
    })
}

impl Arena {
    pub fn set_ball_state(&mut self, state: ffi::BallState) {
        let mut sim_state = SimBallState::DEFAULT;
        sim_state.phys.pos = to_vec3a(state.pos);
        sim_state.phys.vel = to_vec3a(state.vel);
        sim_state.phys.ang_vel = to_vec3a(state.ang_vel);
        sim_state.hs_info = to_hs_info(state.hs_info);
        sim_state.ds_info = to_ds_info(state.ds_info);
        sim_state.last_extra_hit_tick = if state.last_extra_hit_tick_present {
            Some(state.last_extra_hit_tick)
        } else {
            None
        };
        sim_state.tick_count_since_kickoff = state.tick_count_since_kickoff;

        self.inner.set_ball_state(sim_state);
    }

    #[must_use]
    pub fn get_ball_state(&self) -> ffi::BallState {
        let state = self.inner.get_ball_state();
        ffi::BallState {
            pos: from_vec3a(state.phys.pos),
            vel: from_vec3a(state.phys.vel),
            ang_vel: from_vec3a(state.phys.ang_vel),
            hs_info: from_hs_info(state.hs_info),
            ds_info: from_ds_info(state.ds_info),
            last_extra_hit_tick_present: state.last_extra_hit_tick.is_some(),
            last_extra_hit_tick: state.last_extra_hit_tick.unwrap_or(0),
            tick_count_since_kickoff: state.tick_count_since_kickoff,
        }
    }

    pub fn step_tick(&mut self) {
        self.inner.step_tick();
    }
}
