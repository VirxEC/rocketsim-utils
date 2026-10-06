use glam::{Affine3A, IVec3, Vec3A};

use crate::{
    CarBodyConfig, CarControls, CarState, MutatorConfig,
    bullet::{
        box_shape::calculate_local_intertia,
        rigid_body::{Impulse, RigidBody},
    },
    consts::{BT_TO_UU, UU_TO_BT, car as car_consts},
};

/// Asymmetric 8-bit input quantization: negatives scale by 128, positives by 127.
fn quantize_axis_inputs(ctrls: Vec3A) -> Vec3A {
    const UPPER_BOUND: Vec3A = Vec3A::splat(128.0);
    const LOWER_BOUND: Vec3A = Vec3A::splat(127.0);

    let clamped = ctrls.clamp(Vec3A::NEG_ONE, Vec3A::ONE);
    let scale = Vec3A::select(clamped.cmplt(Vec3A::ZERO), UPPER_BOUND, LOWER_BOUND);
    let biased = clamped * scale + UPPER_BOUND;
    let w = biased + biased + Vec3A::splat(0.5);
    let byte = ((w.round().as_ivec3() >> 1i32) & IVec3::splat(0xFF)).as_vec3a();
    let s = byte - UPPER_BOUND;

    Vec3A::select(
        s.cmplt(Vec3A::ZERO),
        s * (1.0 / UPPER_BOUND),
        s / LOWER_BOUND,
    )
}

/// Single-axis form of [`quantize_axis_inputs`].
#[must_use]
fn quantize_axis_input(x: f32) -> f32 {
    let clamped = x.clamp(-1.0, 1.0);
    let y = if clamped < 0.0 {
        (clamped * 128.0).max(-128.0)
    } else {
        (clamped * 127.0).min(127.0)
    };
    let w = ((y + 128.0) + (y + 128.0)) + 0.5;
    let eax = w.round_ties_even() as i32;
    let byte = ((eax >> 1) & 0xFF) as u8;
    let s = (byte as f32) - 128.0;
    if byte < 0x80 {
        s * (1.0 / 128.0)
    } else {
        s / 127.0
    }
}

#[derive(Clone, Copy)]
pub struct Car {
    config: CarBodyConfig,
    state: CarState,
    pub body: RigidBody,
    tick_time: f32,
}

impl Car {
    pub(crate) fn new(
        mutator_config: &MutatorConfig,
        gravity: Vec3A,
        config: CarBodyConfig,
        tick_time: f32,
    ) -> Self {
        let local_inertia =
            calculate_local_intertia(config.hitbox_size * UU_TO_BT * 0.5, mutator_config.car_mass);
        let body = RigidBody::new(mutator_config.car_mass, gravity, local_inertia);

        Self {
            config,
            state: CarState {
                boost: mutator_config.car_spawn_boost_amount,
                ..Default::default()
            },
            body,
            tick_time,
        }
    }

    #[must_use]
    pub const fn get_state(&self) -> &CarState {
        &self.state
    }

    pub const fn set_controls(&mut self, new_controls: CarControls) {
        self.state.controls = new_controls;
    }

    pub(crate) fn set_state(&mut self, state: &CarState) {
        self.body.set_center_of_mass_trans(Affine3A {
            matrix3: state.phys.rot_mat,
            translation: state.phys.pos * UU_TO_BT,
        });

        self.body.lin_vel = state.phys.vel * UU_TO_BT;
        self.body.ang_vel = state.phys.ang_vel;
        self.body.clear_accum_vels();

        self.state = *state;
    }

    fn update_air_torque(&mut self, prev_is_flipping: bool, prev_flip_time: f32) {
        let forward_dir = self.state.get_forward_dir();
        let right_dir = self.state.get_right_dir();
        let up_dir = self.state.get_up_dir();

        let dir_pitch = -right_dir;
        let dir_yaw = up_dir;
        let dir_roll = -forward_dir;

        if self.state.is_flipping && self.state.flip_rel_torque != Vec3A::ZERO {
            let mut rel_dodge_torque = self.state.flip_rel_torque;

            let mut pitch_scale = 1.0;
            if rel_dodge_torque.y != 0.0
                && self.state.controls.pitch != 0.0
                && rel_dodge_torque.y.signum() == self.state.controls.pitch.signum()
                && prev_flip_time >= car_consts::flip::PITCH_CANCEL_GATE_MIN_TIME
            {
                pitch_scale = 1.0 - self.state.controls.pitch.abs().min(1.0);
            }

            rel_dodge_torque.y *= pitch_scale;
            let dodge_torque = rel_dodge_torque * car_consts::flip::TORQUE * self.tick_time;

            self.body.add_impulse(
                Impulse::Angular(self.body.world_trans.matrix3 * dodge_torque),
                false,
                true,
            );
        }

        {
            let [pitch_input, yaw_input, roll_input] =
                quantize_axis_inputs(self.state.controls.pyr()).to_array();
            let mut pitch_torque_scale = 1.0;
            let torque = if pitch_input != 0.0 || yaw_input != 0.0 || roll_input != 0.0 {
                if prev_is_flipping
                    || self.state.has_flipped
                        && prev_flip_time < car_consts::flip::PITCHLOCK_EXTRA_TIME
                {
                    pitch_torque_scale = 0.0;
                }

                pitch_input * dir_pitch * pitch_torque_scale * car_consts::air_control::TORQUE.x
                    + yaw_input * dir_yaw * car_consts::air_control::TORQUE.y
                    + roll_input * dir_roll * car_consts::air_control::TORQUE.z
            } else {
                Vec3A::ZERO
            };

            let ang_vel = self.body.ang_vel;

            let damp_pitch = dir_pitch.dot(ang_vel)
                * car_consts::air_control::DAMPING.x
                * (1.0 - (pitch_input * pitch_torque_scale).abs());
            let damp_yaw =
                dir_yaw.dot(ang_vel) * car_consts::air_control::DAMPING.y * (1.0 - yaw_input.abs());
            let damp_roll = dir_roll.dot(ang_vel) * car_consts::air_control::DAMPING.z;

            let damping = dir_yaw * damp_yaw + dir_pitch * damp_pitch + dir_roll * damp_roll;

            let rb_torque =
                (torque - damping) * car_consts::air_control::TORQUE_APPLY_SCALE * self.tick_time;

            self.body
                .add_impulse(Impulse::Angular(rb_torque), false, true);
        }

        let throttle_scale = if self.state.controls.boost || self.state.is_boosting {
            1.0
        } else {
            quantize_axis_input(self.state.controls.throttle)
        };
        if throttle_scale != 0.0 {
            let throttle_force = forward_dir
                * throttle_scale
                * car_consts::drive::THROTTLE_AIR_ACCEL
                * UU_TO_BT
                * self.tick_time;
            self.body
                .add_impulse(Impulse::Linear(throttle_force), false, true);
        }
    }

    fn update_double_jump_or_flip(
        &mut self,
        mutator_config: &MutatorConfig,
        jump_pressed: bool,
        forward_speed_uu: f32,
    ) {
        self.state.air_time += self.tick_time;

        if self.state.has_jumped && !self.state.is_jumping {
            self.state.air_time_since_jump += self.tick_time;
        } else {
            self.state.air_time_since_jump = 0.0;
        }

        if jump_pressed && self.state.air_time_since_jump < car_consts::jump::DOUBLEJUMP_MAX_DELAY {
            let input_magnitude = self.state.controls.yaw.abs()
                + self.state.controls.pitch.abs()
                + self.state.controls.roll.abs();
            let is_flip_input = input_magnitude >= self.config.dodge_deadzone;

            let can_use = !self.state.is_auto_flipping
                && !self.state.has_double_jumped
                && !self.state.has_flipped
                || if is_flip_input {
                    mutator_config.unlimited_flips
                } else {
                    mutator_config.unlimited_double_jumps
                };

            if can_use {
                self.state.has_jumped = true;

                if is_flip_input {
                    const PYR_SCALE: Vec3A = Vec3A::new(-1.0, 1.0, 1.0);

                    self.state.flip_time = 0.0;
                    self.state.has_flipped = true;
                    self.state.is_flipping = true;

                    let pyr = self.state.controls.pyr() * PYR_SCALE;
                    let mut dodge_dir = quantize_axis_inputs(pyr);
                    dodge_dir.y += dodge_dir.z;

                    if dodge_dir.x.abs() < 0.1 && dodge_dir.y.abs() < 0.1 {
                        dodge_dir = Vec3A::ZERO;
                    } else {
                        dodge_dir = dodge_dir.with_z(0.0).normalize_or_zero();
                    }

                    let deadzone_dodge_dir = Vec3A::select(
                        dodge_dir.abs().cmplt(Vec3A::splat(0.1)),
                        Vec3A::ZERO,
                        dodge_dir,
                    );

                    if deadzone_dodge_dir.length_squared() > f32::EPSILON * f32::EPSILON {
                        self.state.flip_rel_torque = Vec3A::new(-dodge_dir.y, dodge_dir.x, 0.0);

                        let should_dodge_backwards = if forward_speed_uu.abs() < 100. {
                            deadzone_dodge_dir.x.is_sign_negative()
                        } else {
                            deadzone_dodge_dir.x.signum() != forward_speed_uu.signum()
                        };

                        let max_speed_scale_x = if should_dodge_backwards {
                            car_consts::flip::BACKWARD_IMPULSE_MAX_SPEED_SCALE
                        } else {
                            car_consts::flip::FORWARD_IMPULSE_MAX_SPEED_SCALE
                        };

                        let forward_speed_ratio = forward_speed_uu.abs() / car_consts::MAX_SPEED;

                        let mut initial_dodge_vel =
                            deadzone_dodge_dir * car_consts::flip::INITIAL_VEL_SCALE;
                        initial_dodge_vel.x *=
                            ((max_speed_scale_x - 1.) * forward_speed_ratio) + 1.0;
                        initial_dodge_vel.y *= ((car_consts::flip::SIDE_IMPULSE_MAX_SPEED_SCALE
                            - 1.)
                            * forward_speed_ratio)
                            + 1.0;
                        if should_dodge_backwards {
                            initial_dodge_vel.x *= car_consts::flip::BACKWARD_IMPULSE_SCALE_X;
                        }

                        let forward_dir_2d =
                            self.state.get_forward_dir().with_z(0.0).normalize_or_zero();
                        let right_dir_2d = Vec3A::new(-forward_dir_2d.y, forward_dir_2d.x, 0.0);
                        let final_delta_vel = initial_dodge_vel.x * forward_dir_2d
                            + initial_dodge_vel.y * right_dir_2d;

                        self.body.add_impulse(
                            Impulse::Linear(final_delta_vel * UU_TO_BT),
                            false,
                            false,
                        );
                        self.body.limit_vels(
                            car_consts::MAX_SPEED * UU_TO_BT,
                            car_consts::MAX_ANG_SPEED,
                        );
                    }
                } else {
                    let jump_start_force =
                        self.state.get_up_dir() * mutator_config.jump_immediate_force * UU_TO_BT;
                    self.body
                        .add_impulse(Impulse::Linear(jump_start_force), false, false);
                    // Clamp speed after the impulse, as after a dodge impulse.
                    self.body
                        .limit_vels(car_consts::MAX_SPEED * UU_TO_BT, car_consts::MAX_ANG_SPEED);
                    self.state.has_double_jumped = true;
                }
            }
        }

        if self.state.is_flipping {
            let flip_time_pre = self.state.flip_time;
            let still_flipping =
                self.state.has_flipped && flip_time_pre < car_consts::flip::TORQUE_TIME;
            self.state.is_flipping = still_flipping;
            self.state.flip_time = if still_flipping {
                flip_time_pre + self.tick_time
            } else {
                self.tick_time
            };
            if (car_consts::flip::Z_DAMP_START..=car_consts::flip::TORQUE_TIME)
                .contains(&flip_time_pre)
                && (self.body.lin_vel.z < 0.0 || flip_time_pre < car_consts::flip::Z_DAMP_END)
            {
                self.body.lin_vel.z *= 1.0 - car_consts::flip::Z_DAMP_120;
            }
        } else if self.state.has_flipped {
            self.state.flip_time += self.tick_time;
        }
    }

    fn update_boost(&mut self, mutator_config: &MutatorConfig) {
        let accel = mutator_config.boost_accel_air;

        if self.state.is_boosting {
            let cost = mutator_config.boost_used_per_second * self.tick_time;
            self.state.boost = (self.state.boost - cost).max(0.0);
            let depleted = self.state.boost == 0.0;
            let latch_expired = !self.state.controls.boost
                && self.state.boosting_time >= car_consts::boost::MIN_TIME;
            if depleted || latch_expired {
                self.state.is_boosting = false;
                self.state.boosting_time = 0.0;
                self.state.time_since_boosted += self.tick_time;

                if mutator_config.recharge_boost_enabled
                    && self.state.time_since_boosted >= mutator_config.recharge_boost_delay
                {
                    self.state.boost += mutator_config.recharge_boost_per_second * self.tick_time;
                }
            } else {
                self.state.is_boosting = true;
                self.state.boosting_time += self.tick_time;
                self.state.time_since_boosted = 0.0;

                self.body.add_impulse(
                    Impulse::Linear(
                        accel * self.state.get_forward_dir() * UU_TO_BT * self.tick_time,
                    ),
                    false,
                    true,
                );
            }
        } else if self.state.controls.boost && self.state.boost > 0.0 {
            self.state.is_boosting = true;
            self.state.boosting_time += self.tick_time;
            self.state.time_since_boosted = 0.0;

            self.body.add_impulse(
                Impulse::Linear(accel * self.state.get_forward_dir() * UU_TO_BT * self.tick_time),
                false,
                true,
            );
        } else {
            self.state.is_boosting = false;
            self.state.boosting_time = 0.0;
            self.state.time_since_boosted += self.tick_time;

            if mutator_config.recharge_boost_enabled
                && self.state.time_since_boosted >= mutator_config.recharge_boost_delay
            {
                self.state.boost += mutator_config.recharge_boost_per_second * self.tick_time;
            }
        }

        self.state.boost = self.state.boost.clamp(0.0, car_consts::boost::MAX);
    }

    pub(crate) fn pre_tick_update(&mut self, mutator_config: &MutatorConfig) {
        self.state.controls = self.state.controls.clamp();
        let forward_speed_uu = self.body.get_forward_speed() * BT_TO_UU;
        let jump_pressed = self.state.controls.jump && !self.state.prev_controls.jump;

        let prev_is_flipping = self.state.is_flipping;
        let prev_flip_time = self.state.flip_time;
        self.update_double_jump_or_flip(mutator_config, jump_pressed, forward_speed_uu);

        self.update_air_torque(prev_is_flipping, prev_flip_time);
        self.update_boost(mutator_config);
    }

    pub(crate) fn post_tick_update(&mut self) {
        self.state.phys.rot_mat = self.body.world_trans.matrix3;
        self.state.phys.rot_quat = self.body.world_rotation;
        self.state.phys.pos = self.body.world_trans.translation * BT_TO_UU;
        self.state.prev_controls = self.state.controls;
    }

    pub(crate) fn finish_physics_tick(&mut self) {
        self.state.phys.vel = self.body.lin_vel * BT_TO_UU;
        self.state.phys.ang_vel = self.body.ang_vel;
    }

    pub fn step_tick(&mut self, mutator_config: &MutatorConfig) {
        self.body
            .limit_vels(car_consts::MAX_SPEED * UU_TO_BT, car_consts::MAX_ANG_SPEED);
        crate::bullet::quantize::quantize(&mut self.body);

        self.pre_tick_update(mutator_config);

        self.body.step_simulation(self.tick_time);

        self.post_tick_update();
        self.finish_physics_tick();
    }
}
