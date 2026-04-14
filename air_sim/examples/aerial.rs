use std::{f32::consts::TAU, time::Instant};

use air_sim::consts::car;
use glam::{Mat3A, Vec3A};
use indicatif::ProgressIterator;
use rand::{
    RngExt, SeedableRng,
    distr::{StandardUniform, Uniform},
    rngs::SmallRng,
};

#[derive(Clone, Copy)]
struct Rotator {
    pitch: f32,
    yaw: f32,
    roll: f32,
}

impl Rotator {
    fn into_mat3a(self) -> Mat3A {
        let (sp, cp) = self.pitch.sin_cos();
        let (sy, cy) = self.yaw.sin_cos();
        let (sr, cr) = self.roll.sin_cos();

        Mat3A::from_cols(
            Vec3A::new(cp * cy, cp * sy, sp),
            Vec3A::new(cy * sp * sr - cr * sy, sy * sp * sr + cr * cy, -cp * sr),
            Vec3A::new(-cr * cy * sp - sr * sy, -cr * sy * sp + sr * cy, cp * cr),
        )
    }
}

mod reorient {
    //! Credit goes to <https://github.com/uservar/RLReorientTesting/blob/kinematic/src/Methods/Methods.cpp>
    //! for the kinematic-based aerial reorientation code/math

    use std::f32::consts::{PI, TAU};

    use air_sim::{CarControls, consts::car};
    use glam::{Quat, Vec3A};

    const ACCEL_PITCH: f32 =
        car::air_control::TORQUE.to_array()[0] * car::air_control::TORQUE_APPLY_SCALE;
    const ACCEL_YAW: f32 =
        car::air_control::TORQUE.to_array()[1] * car::air_control::TORQUE_APPLY_SCALE;

    fn kinematic_solve_1d_no_damp(err: f32, velocity: f32, accel: f32, dt: f32, tps: f32) -> f32 {
        let inv_eff_accel = 1.0 / accel;

        // Stopping distance: v² / 2a
        let stopping_dist = 0.5 * velocity * velocity.abs() * inv_eff_accel;

        // Predict future error + latency compensation, wrapped to [-pi, pi]
        let future_error = (err + stopping_dist + velocity * dt + PI).rem_euclid(TAU) - PI;

        // scale input
        let gain = inv_eff_accel * tps * tps;
        future_error * gain
    }

    /// Align the car with only the target forward vector,
    /// right/up can be anything.
    pub fn reorient_fwd(
        rot: Quat,
        ang_vel: Vec3A,
        forward: Vec3A,
        dt: f32,
        tps: f32,
        ctrls: &mut CarControls,
    ) {
        const EPS: f32 = 1e-6;

        let inv_rot = rot.inverse();
        let direction = inv_rot * forward;

        let mut rot_axis = Vec3A::new(0.0, -direction.z, direction.y);
        let sine = rot_axis.length();

        if sine < EPS {
            rot_axis = Vec3A::ZERO;
        } else {
            rot_axis /= sine;
        }

        let angle = sine.atan2(direction.x);
        let error_vec = rot_axis * angle;
        let pitch_error = -error_vec.y;
        let yaw_error = error_vec.z;

        let local_ang_vel = inv_rot * ang_vel;
        let pitch_vel = local_ang_vel.y;
        let yaw_vel = -local_ang_vel.z;

        let yaw_input = kinematic_solve_1d_no_damp(yaw_error, yaw_vel, ACCEL_YAW, dt, tps);
        let pitch_input = kinematic_solve_1d_no_damp(pitch_error, pitch_vel, ACCEL_PITCH, dt, tps);

        ctrls.yaw = yaw_input.clamp(-1.0, 1.0);
        ctrls.pitch = pitch_input.clamp(-1.0, 1.0);
        ctrls.roll = 0.0;
    }
}

mod aerial {
    use air_sim::{
        CarControls,
        consts::{GRAVITY_Z, car},
    };
    use glam::{Mat3A, Quat, Vec3A};

    const ANGLE_THREASHOLD: f32 = 0.1;
    const AIR_THROTTLE_ACCEL: f32 = 66.0 + (2.0 / 3.0);

    fn angle(v1: Vec3A, v2: Vec3A) -> f32 {
        v1.dot(v2).acos()
    }

    pub fn simulate_aerial_rsim(
        arena: &mut rocketsim::Arena,
        start_pos: Vec3A,
        target_pos: Vec3A,
        vel: Vec3A,
        ang_vel: Vec3A,
        start_rot: Mat3A,
    ) -> u16 {
        const TPS: f32 = 120.0;
        const DT: f32 = 1.0 / TPS;

        let mut car = rocketsim::CarState::DEFAULT;
        car.pos = start_pos;
        car.vel = vel;
        car.ang_vel = ang_vel;
        car.rot_mat = start_rot;
        arena.set_car_state(0, car);

        let mut num_ticks = 0;
        loop {
            let mut car = *arena.get_car_state(0);
            car.boost = 100.0;
            arena.set_car_state(0, car);

            let car_to_target_dist = (target_pos - car.pos).length();
            dbg!(car_to_target_dist);
            if car_to_target_dist < 10.0 {
                break;
            }

            let est = car_to_target_dist / car.vel.length().max(500.0);

            let xf = car.pos + car.vel * est + 0.5 * Vec3A::new(0.0, 0.0, GRAVITY_Z) * est * est;
            let target_dir = target_pos - xf;

            let mut ctrls = CarControls::DEFAULT;
            crate::reorient::reorient_fwd(
                Quat::from_mat3a(&car.rot_mat),
                car.ang_vel,
                target_dir,
                DT,
                TPS,
                &mut ctrls,
            );

            let direction = target_dir.normalize();
            if angle(car.rot_mat.x_axis, direction) < ANGLE_THREASHOLD {
                let s = target_dir.dot(car.rot_mat.x_axis);
                let delta_v = s / est;

                let accel_diff = car::boost::ACCEL_AIR * car::boost::MIN_TIME;
                if delta_v > accel_diff {
                    ctrls.boost = true;
                    ctrls.throttle = 1.0;
                } else {
                    ctrls.throttle = (delta_v / (AIR_THROTTLE_ACCEL * DT)).clamp(-1.0, 1.0);
                }
            }

            arena.set_car_controls(
                0,
                rocketsim::CarControls {
                    throttle: ctrls.throttle,
                    steer: 0.0,
                    pitch: ctrls.pitch,
                    yaw: ctrls.yaw,
                    roll: ctrls.roll,
                    boost: ctrls.boost,
                    jump: ctrls.jump,
                    handbrake: false,
                },
            );

            num_ticks += 1;
            arena.step_tick();
        }

        num_ticks
    }
}

fn generate_sequence() -> Vec<(Vec3A, Vec3A, Vec3A, Vec3A, Rotator)> {
    let mut rot_rng = SmallRng::seed_from_u64(0).sample_iter(Uniform::new(-TAU, TAU).unwrap());
    let mut ang_rng = SmallRng::seed_from_u64(1).sample_iter(Uniform::new(-TAU, TAU).unwrap());
    let mut ang_vel_rng = SmallRng::seed_from_u64(2)
        .sample_iter(Uniform::new(-car::MAX_ANG_SPEED, car::MAX_ANG_SPEED).unwrap());
    let mut pos_rng =
        SmallRng::seed_from_u64(3).sample_iter(Uniform::new(-4000.0, 4000.0).unwrap());
    let mut end_rng_dir = SmallRng::seed_from_u64(4).sample_iter(StandardUniform);
    let mut end_rng_mag =
        SmallRng::seed_from_u64(5).sample_iter(Uniform::new(0.0, 4000.0).unwrap());
    let mut vel_rng = SmallRng::seed_from_u64(6)
        .sample_iter(Uniform::new(-car::MAX_SPEED, car::MAX_SPEED).unwrap());

    // (0..1_000)
    (0..1)
        .map(|_| {
            let start_pos = Vec3A::new(
                pos_rng.next().unwrap(),
                pos_rng.next().unwrap(),
                pos_rng.next().unwrap() / 2.0 + 2000.0,
            );
            let target_pos_dir = Vec3A::new(
                end_rng_dir.next().unwrap(),
                end_rng_dir.next().unwrap(),
                end_rng_dir.next().unwrap(),
            )
            .normalize();
            let mut target_pos = start_pos + target_pos_dir * end_rng_mag.next().unwrap();
            target_pos.z = target_pos.z.max(0.0);

            let vel = Vec3A::new(
                vel_rng.next().unwrap(),
                vel_rng.next().unwrap(),
                vel_rng.next().unwrap(),
            )
            .normalize()
                * vel_rng.next().unwrap();

            let ang_vel = Vec3A::new(
                ang_rng.next().unwrap(),
                ang_rng.next().unwrap(),
                ang_rng.next().unwrap(),
            )
            .normalize()
                * ang_vel_rng.next().unwrap();

            let start_rot = Rotator {
                pitch: rot_rng.next().unwrap(),
                yaw: rot_rng.next().unwrap(),
                roll: rot_rng.next().unwrap(),
            };

            (start_pos, target_pos, vel, ang_vel, start_rot)
        })
        .collect()
}

fn main() {
    let mut arena = rocketsim::Arena::new(rocketsim::GameMode::TheVoid);
    arena.add_car(rocketsim::Team::Blue, rocketsim::CarBodyConfig::OCTANE);

    let sequence = generate_sequence();
    let num = sequence.len();

    for tps in [120, 90, 60] {
        let mut num_frames = Vec::with_capacity(num);

        let start_time = Instant::now();
        for (start_pos, target_pos, vel, ang_vel, start_rot) in sequence.iter().copied().progress()
        {
            let start_orientation = start_rot.into_mat3a();
            let frames_elapsed = aerial::simulate_aerial_rsim(
                &mut arena,
                start_pos,
                target_pos,
                vel,
                ang_vel,
                start_orientation,
            );
            num_frames.push(u32::from(frames_elapsed));
        }

        let irl_time_elapsed = start_time.elapsed().as_secs_f64();

        num_frames.sort_unstable();
        let tps = f64::from(tps);
        let total_frames = f64::from(num_frames.iter().sum::<u32>());
        let sim_time_elapsed = total_frames / tps;
        let mean = total_frames / num as f64 / tps;
        let median = num_frames[num / 2] as f64 / tps;
        let max = num_frames[num - 1] as f64 / tps;

        let speedup = sim_time_elapsed / irl_time_elapsed;
        let time_per_sim = irl_time_elapsed * 1_000_000.0 / num as f64;
        println!(
            "tps={tps:.0} | Finished simulating {sim_time_elapsed:.2}s in {irl_time_elapsed:.2}s ({speedup:.0}x; {time_per_sim:.1}mus per), {mean:.4}s mean, {median:.4}s median, {max:.4}s max"
        );
    }
}
