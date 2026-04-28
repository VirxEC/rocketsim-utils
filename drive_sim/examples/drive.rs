//! Simulate driving from one place to another on the floor.
//!
//! Car control is based off of RLUtilities, and car path finding is done using [`dubins_paths`].

use std::{f32::consts::TAU, time::Instant};

use drive_sim::{Arena, CarControls, CarState, GameMode, PhysState, consts::car::spawn::REST_Z};
use dubins_paths::f32::{DubinsPath, PosRot};
use glam::{Mat3A, Vec3A};
use indicatif::ProgressIterator;
use rand::{RngExt, SeedableRng, distr::Uniform, rngs::SmallRng};

fn mat3a_from_direction(direction: Vec3A, up: Vec3A) -> Mat3A {
    let forward = direction.normalize();
    let up = forward.cross(up.cross(forward)).normalize();
    let right = up.cross(forward);

    Mat3A::from_cols(forward, right, up)
}

struct GlamPhysics {
    pub rot: Mat3A,
    pub local_ang_vel: Vec3A,
    pub pos: Vec3A,
    pub local_vel: Vec3A,
}

impl GlamPhysics {
    fn local_pos(&self, vec: Vec3A) -> Vec3A {
        self.rot.mul_transpose_vec3a(vec - self.pos)
    }
}

const fn curvature(v: f32) -> f32 {
    if 0.0 <= v && v < 500.0 {
        0.006_900 - 5.84e-6 * v
    } else if 500.0 <= v && v < 1000.0 {
        0.005_610 - 3.26e-6 * v
    } else if 1000.0 <= v && v < 1500.0 {
        0.004_300 - 1.95e-6 * v
    } else if 1500.0 <= v && v < 1750.0 {
        0.003_025 - 1.1e-6 * v
    } else if 1750.0 <= v && v < 2500.0 {
        0.001_800 - 4e-7 * v
    } else {
        0.0
    }
}

const fn turn_radius(v: f32) -> f32 {
    if v == 0.0 { 0.0 } else { 1.0 / curvature(v) }
}

fn generate_sequence() -> Vec<(PosRot, PosRot)> {
    let mut rng = SmallRng::seed_from_u64(0).sample_iter(Uniform::new(-3000.0, 3000.0).unwrap());
    let mut rot_rng = SmallRng::seed_from_u64(1).sample_iter(Uniform::new(-TAU, TAU).unwrap());

    // (0..1)
    (0..20_000)
        .map(|_| {
            let start = PosRot::from_floats(
                rng.next().unwrap(),
                rng.next().unwrap(),
                rot_rng.next().unwrap(),
            );

            let end = PosRot::from_floats(
                rng.next().unwrap(),
                rng.next().unwrap(),
                rot_rng.next().unwrap(),
            );

            (start, end)
        })
        .collect()
}

/// From RLUtilities
fn control_pid(angle: f32, rate: f32, distance: f32) -> f32 {
    (3.4 * angle + 0.225 * rate - 0.0015 * distance).clamp(-1.0, 1.0)
}

fn throttle_acceleration(car_velocity_x: f32) -> f32 {
    match car_velocity_x.abs() {
        x if x >= 1410.0 => 0.0,
        x if x < 1400.0 => (-36.0 / 35.0) * x + 1600.0,
        x => -16.0 * (x - 1400.0) + 160.0,
    }
}

fn get_throttle_and_boost(
    boost_accel: f32,
    target_speed: f32,
    car_speed: f32,
    angle_to_target: f32,
    handbrake: bool,
) -> (f32, bool) {
    const COAST_ACC: f32 = 525.0;
    const BRAKE_ACC: f32 = 3500.0;
    const REACTION_TIME: f32 = 0.04;
    const BRAKE_COAST_TRANSITION: f32 = -(0.45 * BRAKE_ACC + 0.55 * COAST_ACC);
    const COASTING_THROTTLE_TRANSITION: f32 = -0.5 * COAST_ACC;

    let t = target_speed - car_speed;
    let mut acceleration = t / REACTION_TIME;
    if car_speed < 0.0 {
        acceleration *= -1.0;
    }

    let throttle_accel = throttle_acceleration(car_speed);
    let throttle_boost_transition = 1.0 * throttle_accel + 0.5 * boost_accel;

    let mut throttle = 0.0;
    let mut boost = false;

    if acceleration <= BRAKE_COAST_TRANSITION {
        throttle = -1.0;
    } else if BRAKE_COAST_TRANSITION < acceleration && acceleration < COASTING_THROTTLE_TRANSITION {
        // coast
    } else if COASTING_THROTTLE_TRANSITION <= acceleration
        && acceleration <= throttle_boost_transition
    {
        throttle = if throttle_accel == 0.0 {
            1.0
        } else {
            (acceleration / throttle_accel).clamp(0.015, 1.0)
        };
    } else if throttle_boost_transition < acceleration {
        throttle = 1.0;
        boost = !handbrake && t > 0.0 && angle_to_target.abs() < 0.1;
    }

    if car_speed < 0.0 {
        throttle *= -1.0;
    }

    (throttle, boost)
}

fn main() {
    const STEER_REACTION_TIME: f32 = 0.3;
    const BOOST_ACCEL: f32 = 2975. / 3.;

    let sequence = generate_sequence();
    let num = sequence.len();

    for tps in [120, 90, 60] {
        let trunc_num_ticks = 6 * u32::from(tps);

        let mut num_frames = Vec::with_capacity(num);
        let mut num_ran_wide = 0;
        let mut num_runs_truncated = 0;
        let mut num_missed = 0;
        let mut num_ran_wide_and_truncated = 0;
        let mut num_ran_wide_and_missed = 0;

        let mut arena = Arena::new(GameMode::Soccar, tps);

        let start_time = Instant::now();
        for (start, end) in sequence.iter().copied().progress() {
            let max_speed = 2300.0;
            let rho = turn_radius(max_speed);
            let Ok(path) = DubinsPath::shortest_from(start, end, rho) else {
                panic!("Couldn't find path between {start:?} and {end:?}");
            };
            let sampler = path.get_path_sampler();
            let path_info = path.get_path_info();

            // simulate following the path
            let car = CarState {
                phys: PhysState {
                    pos: start.pos().extend(REST_Z).into(),
                    vel: Vec3A::ZERO,
                    ang_vel: Vec3A::ZERO,
                    rot_mat: mat3a_from_direction(
                        Vec3A::new(start.rot().cos(), start.rot().sin(), 0.0),
                        Vec3A::Z,
                    ),
                },
                boost: 100.0,
                ..CarState::DEFAULT
            };
            arena.set_car_state(car);

            let mut frames_elapsed = 0;
            let mut ran_wide = false;
            let mut missed = false;
            while frames_elapsed < trunc_num_ticks {
                let car = arena.get_car_state();

                let phys = GlamPhysics {
                    pos: car.pos,
                    local_vel: car.rot_mat.mul_transpose_vec3a(car.vel),
                    local_ang_vel: car.rot_mat.mul_transpose_vec3a(car.ang_vel),
                    rot: car.rot_mat,
                };

                let local_end = phys.local_pos(end.pos().extend(0.0).into());
                if local_end.x.abs() < 20.0 && local_end.y.abs() < 30.0 {
                    break;
                }

                frames_elapsed += 1;

                let path_length = path.length();
                let current_target_distance =
                    path_info.est_distance_traveled(phys.pos.truncate().into());
                if (current_target_distance - path_length).abs() < f32::EPSILON
                    && local_end.x.is_sign_negative()
                {
                    missed = true;
                    break;
                }

                let current_point = sampler.sample(current_target_distance).pos();
                let local_current_point = phys.local_pos(current_point.extend(0.0).into());
                let distance = local_current_point.y;
                if distance > 45.0 {
                    ran_wide = true;
                }

                let limit = path_length;
                let additional_distance = phys.local_vel.x.max(500.) * STEER_REACTION_TIME;
                let target_distance = current_target_distance + additional_distance;
                let point_on_path: Vec3A = sampler
                    .sample(target_distance.min(limit))
                    .pos()
                    .extend(0.0)
                    .into();

                let drive_target = point_on_path;
                let mut local_target_pos = phys.local_pos(drive_target);

                let mut ctrls = CarControls::DEFAULT;
                local_target_pos.z = 0.0;

                let target_angle = local_target_pos.y.atan2(local_target_pos.x);
                ctrls.steer = control_pid(target_angle, phys.local_ang_vel.z, distance);

                (ctrls.throttle, ctrls.boost) = get_throttle_and_boost(
                    BOOST_ACCEL,
                    max_speed,
                    phys.local_vel.x,
                    target_angle,
                    ctrls.handbrake,
                );

                arena.set_car_controls(ctrls);
                arena.step_tick();
            }

            num_frames.push(frames_elapsed);
            if ran_wide {
                num_ran_wide += 1;
            }
            if frames_elapsed >= trunc_num_ticks {
                num_runs_truncated += 1;
            }
            if missed {
                num_missed += 1;
            }
            if ran_wide && frames_elapsed >= trunc_num_ticks {
                num_ran_wide_and_truncated += 1;
            }
            if ran_wide && missed {
                num_ran_wide_and_missed += 1;
            }
        }

        let irl_time_elapsed = start_time.elapsed().as_secs_f64();

        num_frames.sort_unstable();
        let ftps = f64::from(tps);
        let total_frames = f64::from(num_frames.iter().sum::<u32>());
        let sim_time_elapsed = total_frames / ftps;
        let mean = total_frames / num as f64 / ftps;
        let median = f64::from(num_frames[num / 2]) / ftps;
        let max = f64::from(num_frames[num - 1]) / ftps;

        let speedup = sim_time_elapsed / irl_time_elapsed;
        let time_per_sim = irl_time_elapsed * 1_000_000.0 / num as f64;
        println!(
            "tps={tps} | Finished simulating {sim_time_elapsed:.2}s in {irl_time_elapsed:.2}s ({speedup:.0}x; {time_per_sim:.1}mus per), {mean:.4}s mean, {median:.4}s median, {max:.4}s max"
        );
        println!(
            "\t{num_ran_wide} ran wide ({num_ran_wide_and_truncated} truncated, {num_ran_wide_and_missed} missed), {num_runs_truncated} truncated, {num_missed} missed"
        );
    }
}
