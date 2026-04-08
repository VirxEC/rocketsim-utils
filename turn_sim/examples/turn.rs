use std::{f32::consts::TAU, time::Instant};

use glam::{EulerRot, Mat3A, Quat, Vec3A};
use indicatif::ProgressIterator;
use rand::{
    RngExt, SeedableRng,
    distr::{StandardUniform, Uniform},
    rngs::SmallRng,
};
use turn_sim::consts::car;

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

    fn from_mat3a(mat: Mat3A) -> Self {
        Self {
            pitch: mat.x_axis.z.atan2(mat.x_axis.x.hypot(mat.x_axis.y)),
            yaw: mat.x_axis.y.atan2(mat.x_axis.x),
            roll: (-mat.y_axis.z).atan2(mat.z_axis.z),
        }
    }
}

mod reorient {
    //! Credit goes to <https://github.com/uservar/RLReorientTesting/blob/kinematic/src/Methods/Methods.cpp>
    //! for the kinematic-based aerial reorientation code/math

    use std::f32::consts::{PI, TAU};

    use glam::{Quat, Vec3A};
    use turn_sim::{Car, car_controls::CarControls, consts::car};

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
    fn reorient_fwd(
        rot: Quat,
        ang_vel: Vec3A,
        forward: Vec3A,
        dt: f32,
        tps: f32,
        ctrls: &mut CarControls,
    ) -> bool {
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

        error_vec.with_x(0.0).length() < 0.01 && local_ang_vel.with_x(0.0).length() < 0.015
    }

    pub fn simulate_reorient_fwd(ang_vel: Vec3A, rot: Quat, forward: Vec3A, tps: u16) -> u16 {
        let tps = f32::from(tps);
        let dt = 1.0 / tps;

        let mut num_frames = 0;
        let mut car = Car { ang_vel, rot };
        let mut ctrls = CarControls::default();

        while !reorient_fwd(car.rot, car.ang_vel, forward, dt, tps, &mut ctrls) {
            car.step_turn(ctrls, dt);
            num_frames += 1;
        }

        num_frames
    }
}

fn generate_sequence() -> Vec<(Rotator, Rotator, Vec3A)> {
    let mut rng = SmallRng::seed_from_u64(0).sample_iter(StandardUniform);
    let mut ang_rng = SmallRng::seed_from_u64(1).sample_iter(Uniform::new(-TAU, TAU).unwrap());
    let mut ang_vel_rng = SmallRng::seed_from_u64(2)
        .sample_iter(Uniform::new(-car::MAX_ANG_SPEED, car::MAX_ANG_SPEED).unwrap());

    // (0..1)
    (0..300_000)
        .map(|_| {
            let start = Rotator {
                pitch: rng.next().unwrap() * TAU,
                yaw: rng.next().unwrap() * TAU,
                roll: rng.next().unwrap() * TAU,
            };

            // get end by rotating start by a random angle in a random axis
            let axis = Vec3A::new(
                rng.next().unwrap(),
                rng.next().unwrap(),
                rng.next().unwrap(),
            )
            .normalize();
            let angle = ang_rng.next().unwrap();
            let rot_mat = start.into_mat3a();
            let delta_rot = Mat3A::from_axis_angle(axis.into(), angle);
            let end_mat = delta_rot * rot_mat;
            let end = Rotator::from_mat3a(end_mat);

            (
                start,
                end,
                Vec3A::new(
                    rng.next().unwrap(),
                    rng.next().unwrap(),
                    rng.next().unwrap(),
                )
                .normalize()
                    * ang_vel_rng.next().unwrap(),
            )
        })
        .collect()
}

fn main() {
    let sequence = generate_sequence();
    let num = sequence.len();

    for tps in [120, 90, 60] {
        let mut num_frames = Vec::with_capacity(num);

        let start_time = Instant::now();
        for (start_rot, target_rot, ang_vel) in sequence.iter().copied().progress() {
            let target_orientation = target_rot.into_mat3a();
            let frames_elapsed = reorient::simulate_reorient_fwd(
                ang_vel,
                Quat::from_euler(
                    EulerRot::ZYX,
                    start_rot.yaw,
                    -start_rot.pitch,
                    -start_rot.roll,
                ),
                target_orientation.x_axis,
                tps,
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
