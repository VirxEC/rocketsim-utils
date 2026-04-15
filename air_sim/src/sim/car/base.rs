use glam::{Affine3A, Vec3A};

use crate::{
    CarBodyConfig, CarControls, CarState, MutatorConfig,
    bullet::{box_shape::calculate_local_intertia, rigid_body::RigidBody},
    consts::{BT_TO_UU, UU_TO_BT, car as car_consts},
};

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

        self.state = *state;
    }

    fn update_air_torque(&mut self) {
        let forward_dir = self.state.get_forward_dir();
        let right_dir = self.state.get_right_dir();
        let up_dir = self.state.get_up_dir();

        let dir_pitch = -right_dir;
        let dir_yaw = up_dir;
        let dir_roll = -forward_dir;

        if self.state.is_flipping {
            self.state.is_flipping =
                self.state.has_flipped && self.state.flip_time < car_consts::flip::TORQUE_TIME;
        }

        let mut do_air_control = false;
        if self.state.is_flipping {
            if self.state.flip_rel_torque == Vec3A::ZERO {
                do_air_control = true;
            } else {
                let mut rel_dodge_torque = self.state.flip_rel_torque;

                let mut pitch_scale = 1.0;
                if rel_dodge_torque.y != 0.0
                    && self.state.controls.pitch != 0.0
                    && rel_dodge_torque.y.signum() == self.state.controls.pitch.signum()
                {
                    pitch_scale = 1.0 - self.state.controls.pitch.abs().min(1.0);
                    do_air_control = true;
                }

                rel_dodge_torque.y *= pitch_scale;
                let dodge_torque = rel_dodge_torque
                    * const { Vec3A::new(car_consts::flip::TORQUE_X, car_consts::flip::TORQUE_Y, 0.0) };

                let rb_torque = self.body.inv_inertia_tensor_world.inverse()
                    * self.body.world_trans.matrix3
                    * dodge_torque;
                self.body.apply_torque(rb_torque);
            }
        } else {
            do_air_control = true;
        }

        if do_air_control {
            let mut pitch_torque_scale = 1.0;
            let torque = if self.state.controls.pitch != 0.0
                || self.state.controls.yaw != 0.0
                || self.state.controls.roll != 0.0
            {
                if self.state.is_flipping
                    || self.state.has_flipped
                        && self.state.flip_time
                            < const {
                                car_consts::flip::TORQUE_TIME
                                    + car_consts::flip::PITCHLOCK_EXTRA_TIME
                            }
                {
                    pitch_torque_scale = 0.0;
                }

                self.state.controls.pitch
                    * dir_pitch
                    * pitch_torque_scale
                    * car_consts::air_control::TORQUE.x
                    + self.state.controls.yaw * dir_yaw * car_consts::air_control::TORQUE.y
                    + self.state.controls.roll * dir_roll * car_consts::air_control::TORQUE.z
            } else {
                Vec3A::ZERO
            };

            let ang_vel = self.body.ang_vel;

            let damp_pitch = dir_pitch.dot(ang_vel)
                * car_consts::air_control::DAMPING.x
                * (1.0 - (self.state.controls.pitch * pitch_torque_scale).abs());
            let damp_yaw = dir_yaw.dot(ang_vel)
                * car_consts::air_control::DAMPING.y
                * (1.0 - self.state.controls.yaw.abs());
            let damp_roll = dir_roll.dot(ang_vel) * car_consts::air_control::DAMPING.z;

            let damping = dir_yaw * damp_yaw + dir_pitch * damp_pitch + dir_roll * damp_roll;

            let rb_torque = self.body.inv_inertia_tensor_world.inverse()
                * (torque - damping)
                * car_consts::air_control::TORQUE_APPLY_SCALE;
            self.body.apply_torque(rb_torque);
        }

        if self.state.controls.throttle != 0.0 {
            self.body.apply_central_force(
                forward_dir
                    * self.state.controls.throttle
                    * const { car_consts::drive::THROTTLE_AIR_ACCEL * UU_TO_BT * car_consts::MASS_BT },
            );
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

            let can_use = !self.state.has_double_jumped && !self.state.has_flipped
                || if is_flip_input {
                    mutator_config.unlimited_flips
                } else {
                    mutator_config.unlimited_double_jumps
                };

            if can_use {
                if is_flip_input {
                    self.state.flip_time = 0.0;
                    self.state.has_flipped = true;
                    self.state.is_flipping = true;

                    let forward_speed_ratio = forward_speed_uu.abs() / car_consts::MAX_SPEED;
                    let mut dodge_dir = Vec3A::new(
                        -self.state.controls.pitch,
                        self.state.controls.yaw + self.state.controls.roll,
                        0.0,
                    );

                    if dodge_dir.x.abs() < 0.1 && dodge_dir.y.abs() < 0.1 {
                        dodge_dir = Vec3A::ZERO;
                    } else {
                        dodge_dir = dodge_dir.normalize_or_zero();
                    }

                    self.state.flip_rel_torque = Vec3A::new(-dodge_dir.y, dodge_dir.x, 0.0);

                    if dodge_dir.x.abs() < 0.1 {
                        dodge_dir.x = 0.0;
                    }

                    if dodge_dir.y.abs() < 0.1 {
                        dodge_dir.y = 0.0;
                    }

                    if dodge_dir.length_squared() > const { f32::EPSILON * f32::EPSILON } {
                        let should_dodge_backwards = if forward_speed_uu.abs() < 100. {
                            dodge_dir.x.is_sign_negative()
                        } else {
                            dodge_dir.x.signum() != forward_speed_uu.signum()
                        };

                        let max_speed_scale_x = if should_dodge_backwards {
                            car_consts::flip::BACKWARD_IMPULSE_MAX_SPEED_SCALE
                        } else {
                            car_consts::flip::FORWARD_IMPULSE_MAX_SPEED_SCALE
                        };

                        let mut initial_dodge_vel = dodge_dir * car_consts::flip::INITIAL_VEL_SCALE;
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

                        self.body.apply_central_impulse(
                            final_delta_vel * const { UU_TO_BT * car_consts::MASS_BT },
                        );
                    }
                } else {
                    let jump_start_force = self.state.get_up_dir()
                        * const { car_consts::jump::IMMEDIATE_FORCE * UU_TO_BT * car_consts::MASS_BT };
                    self.body.apply_central_impulse(jump_start_force);
                    self.state.has_double_jumped = true;
                }
            }
        }

        if self.state.is_flipping {
            self.state.flip_time += self.tick_time;
            if self.state.flip_time <= car_consts::flip::TORQUE_TIME
                && self.state.flip_time >= car_consts::flip::Z_DAMP_START
                && (self.body.lin_vel.z < 0.0
                    || self.state.flip_time < car_consts::flip::Z_DAMP_END)
            {
                self.body.lin_vel.z *= 1.0 - car_consts::flip::Z_DAMP_120;
            }
        } else if self.state.has_flipped {
            self.state.flip_time += self.tick_time;
        }
    }

    fn update_boost(&mut self, mutator_config: &MutatorConfig) {
        self.state.is_boosting = if self.state.boost > 0.0 {
            self.state.controls.boost
                || (self.state.is_boosting
                    && self.state.boosting_time < car_consts::boost::MIN_TIME)
        } else {
            false
        };

        if self.state.is_boosting {
            self.state.boosting_time += self.tick_time;
            self.state.time_since_boosted = 0.0;
            self.state.boost -= mutator_config.boost_used_per_second * self.tick_time;

            self.body.apply_central_force(
                mutator_config.boost_accel_air
                    * self.state.get_forward_dir()
                    * (UU_TO_BT * car_consts::MASS_BT),
            );
        } else {
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

        self.update_air_torque();
        self.update_double_jump_or_flip(mutator_config, jump_pressed, forward_speed_uu);
        self.update_boost(mutator_config);
    }

    pub(crate) fn post_tick_update(&mut self) {
        self.state.phys.rot_mat = self.body.world_trans.matrix3;
        self.state.phys.rot_quat = self.body.world_rotation;
        self.state.phys.pos = self.body.world_trans.translation * BT_TO_UU;
        self.state.prev_controls = self.state.controls;
    }

    pub(crate) fn finish_physics_tick(&mut self) {
        const MAX_SPEED: f32 = car_consts::MAX_SPEED * UU_TO_BT;

        let lin_vel_len_sq = self.body.lin_vel.length_squared();
        if lin_vel_len_sq > const { MAX_SPEED * MAX_SPEED } {
            self.body.lin_vel = self.body.lin_vel / lin_vel_len_sq.sqrt() * MAX_SPEED;
        }

        let ang_vel_len_sq = self.body.ang_vel.length_squared();
        if ang_vel_len_sq > const { car_consts::MAX_ANG_SPEED * car_consts::MAX_ANG_SPEED } {
            self.body.ang_vel =
                self.body.ang_vel / ang_vel_len_sq.sqrt() * car_consts::MAX_ANG_SPEED;
        }

        self.state.phys.vel = self.body.lin_vel * BT_TO_UU;
        self.state.phys.ang_vel = self.body.ang_vel;
    }

    pub fn step_tick(&mut self, mutator_config: &MutatorConfig) {
        self.pre_tick_update(mutator_config);

        self.body.step_simulation(self.tick_time);

        self.post_tick_update();
        self.finish_physics_tick();
    }
}
