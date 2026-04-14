use std::ops::{Deref, DerefMut};

use glam::{Affine3A, Vec3A};

// Shorthand using aliases for constants
use crate::{
    CarBodyConfig, CarControls, CarState, MutatorConfig,
    bullet::{
        collision::box_shape::BoxShape,
        dynamics::{
            discrete_dynamics_world::DiscreteDynamicsWorld,
            rigid_body::{RigidBody, RigidBodyConstructionInfo},
        },
        linear_math::Mat3AExt,
    },
    consts::{BT_TO_UU, TICK_TIME, UU_TO_BT, car as car_consts},
    sim::car::car_info::CarInfo,
};

pub struct Car {
    pub(crate) info: CarInfo,
    pub(crate) rigid_body_idx: usize,
    pub(crate) vel_impulse_cache: Vec3A,
    pub(crate) state: CarState,
}

impl Deref for Car {
    type Target = CarInfo;
    fn deref(&self) -> &Self::Target {
        &self.info
    }
}
impl DerefMut for Car {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.info
    }
}

impl Car {
    pub(crate) fn new(
        idx: usize,
        bullet_world: &mut DiscreteDynamicsWorld,
        mutator_config: &MutatorConfig,
        config: CarBodyConfig,
    ) -> Self {
        let child_hitbox_shape = BoxShape::new(config.hitbox_size * UU_TO_BT * 0.5);
        let local_inertia = child_hitbox_shape.calculate_local_intertia(car_consts::MASS_BT);

        let mut rb_info = RigidBodyConstructionInfo::new(car_consts::MASS_BT);
        rb_info.friction = car_consts::BASE_COEFS.friction;
        rb_info.restitution = car_consts::BASE_COEFS.restitution;
        rb_info.start_world_trans = Affine3A::IDENTITY;
        rb_info.local_inertia = local_inertia;

        let body = RigidBody::new(rb_info);
        let rigid_body_idx = bullet_world.add_rigid_body(body);

        Self {
            info: CarInfo { idx, config },
            rigid_body_idx,
            vel_impulse_cache: Vec3A::ZERO,
            state: CarState {
                boost: mutator_config.car_spawn_boost_amount,
                ..Default::default()
            },
        }
    }

    pub const fn get_state(&self) -> &CarState {
        &self.state
    }

    pub const fn get_config(&self) -> &CarBodyConfig {
        &self.info.config
    }

    pub const fn set_controls(&mut self, new_controls: CarControls) {
        self.state.controls = new_controls;
    }

    pub(crate) fn set_state(&mut self, rb: &mut RigidBody, state: &CarState) {
        debug_assert_eq!(rb.world_array_idx, self.rigid_body_idx);

        rb.set_world_trans(Affine3A {
            matrix3: state.phys.rot_mat,
            translation: state.phys.pos * UU_TO_BT,
        });

        rb.lin_vel = state.phys.vel * UU_TO_BT;
        rb.ang_vel = state.phys.ang_vel;
        rb.update_inertia_tensor();

        self.vel_impulse_cache = Vec3A::ZERO;
        self.state = *state;
    }

    fn update_air_torque(&mut self, rb: &mut RigidBody) {
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

                let rb_torque = rb.inv_inertia_tensor_world.bullet_inverse()
                    * rb.get_world_trans().matrix3
                    * dodge_torque;
                rb.apply_torque(rb_torque);
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

            let ang_vel = rb.ang_vel;

            let damp_pitch = dir_pitch.dot(ang_vel)
                * car_consts::air_control::DAMPING.x
                * (1.0 - (self.state.controls.pitch * pitch_torque_scale).abs());
            let damp_yaw = dir_yaw.dot(ang_vel)
                * car_consts::air_control::DAMPING.y
                * (1.0 - self.state.controls.yaw.abs());
            let damp_roll = dir_roll.dot(ang_vel) * car_consts::air_control::DAMPING.z;

            let damping = dir_yaw * damp_yaw + dir_pitch * damp_pitch + dir_roll * damp_roll;

            let rb_torque = rb.inv_inertia_tensor_world.bullet_inverse()
                * (torque - damping)
                * car_consts::air_control::TORQUE_APPLY_SCALE;
            rb.apply_torque(rb_torque);
        }

        if self.state.controls.throttle != 0.0 {
            rb.apply_central_force(
                forward_dir
                    * self.state.controls.throttle
                    * const { car_consts::drive::THROTTLE_AIR_ACCEL * UU_TO_BT * car_consts::MASS_BT },
            );
        }
    }

    fn update_double_jump_or_flip(
        &mut self,
        rb: &mut RigidBody,
        mutator_config: &MutatorConfig,
        jump_pressed: bool,
        forward_speed_uu: f32,
    ) {
        self.state.air_time += TICK_TIME;

        if self.state.has_jumped && !self.state.is_jumping {
            self.state.air_time_since_jump += TICK_TIME;
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

                        rb.apply_central_impulse(
                            final_delta_vel * const { UU_TO_BT * car_consts::MASS_BT },
                        );
                    }
                } else {
                    let jump_start_force = self.state.get_up_dir()
                        * const { car_consts::jump::IMMEDIATE_FORCE * UU_TO_BT * car_consts::MASS_BT };
                    rb.apply_central_impulse(jump_start_force);
                    self.state.has_double_jumped = true;
                }
            }
        }

        if self.state.is_flipping {
            self.state.flip_time += TICK_TIME;
            if self.state.flip_time <= car_consts::flip::TORQUE_TIME
                && self.state.flip_time >= car_consts::flip::Z_DAMP_START
                && (rb.lin_vel.z < 0.0 || self.state.flip_time < car_consts::flip::Z_DAMP_END)
            {
                rb.lin_vel.z *= 1.0 - car_consts::flip::Z_DAMP_120;
            }
        } else if self.state.has_flipped {
            self.state.flip_time += TICK_TIME;
        }
    }

    fn update_boost(&mut self, rb: &mut RigidBody, mutator_config: &MutatorConfig) {
        self.state.is_boosting = if self.state.boost > 0.0 {
            self.state.controls.boost
                || (self.state.is_boosting
                    && self.state.boosting_time < car_consts::boost::MIN_TIME)
        } else {
            false
        };

        if self.state.is_boosting {
            self.state.boosting_time += TICK_TIME;
            self.state.time_since_boosted = 0.0;
            self.state.boost -= mutator_config.boost_used_per_second * TICK_TIME;

            rb.apply_central_force(
                mutator_config.boost_accel_air
                    * self.state.get_forward_dir()
                    * (UU_TO_BT * car_consts::MASS_BT),
            );
        } else {
            self.state.boosting_time = 0.0;
            self.state.time_since_boosted += TICK_TIME;

            if mutator_config.recharge_boost_enabled
                && self.state.time_since_boosted >= mutator_config.recharge_boost_delay
            {
                self.state.boost += mutator_config.recharge_boost_per_second * TICK_TIME;
            }
        }

        self.state.boost = self.state.boost.clamp(0.0, car_consts::boost::MAX);
    }

    pub(crate) fn pre_tick_update(
        &mut self,
        collision_world: &mut DiscreteDynamicsWorld,
        mutator_config: &MutatorConfig,
    ) {
        let rb = &mut collision_world.bodies_mut()[self.rigid_body_idx];

        self.state.controls = self.state.controls.clamp();
        let forward_speed_uu = rb.get_forward_speed() * BT_TO_UU;
        let jump_pressed = self.state.controls.jump && !self.state.prev_controls.jump;

        self.update_air_torque(rb);
        self.update_double_jump_or_flip(rb, mutator_config, jump_pressed, forward_speed_uu);
        self.update_boost(rb, mutator_config);
    }

    pub(crate) fn post_tick_update(&mut self, rb: &RigidBody) {
        debug_assert_eq!(rb.world_array_idx, self.rigid_body_idx);

        self.state.phys.rot_mat = rb.get_world_trans().matrix3;

        let speed_squared = (rb.lin_vel * BT_TO_UU).length_squared();
        self.state.is_supersonic = speed_squared
            >= if self.state.is_supersonic
                && self.state.supersonic_time < car_consts::supersonic::MAINTAIN_MAX_TIME
            {
                const {
                    car_consts::supersonic::MAINTAIN_MIN_SPEED
                        * car_consts::supersonic::MAINTAIN_MIN_SPEED
                }
            } else {
                const { car_consts::supersonic::START_SPEED * car_consts::supersonic::START_SPEED }
            };

        if self.state.is_supersonic {
            self.state.supersonic_time += TICK_TIME;
        } else {
            self.state.supersonic_time = 0.0;
        }

        self.state.prev_controls = self.state.controls;
    }

    pub(crate) fn finish_physics_tick(&mut self, rb: &mut RigidBody) {
        const MAX_SPEED: f32 = car_consts::MAX_SPEED * UU_TO_BT;
        debug_assert_eq!(rb.world_array_idx, self.rigid_body_idx);

        if self.vel_impulse_cache != Vec3A::ZERO {
            rb.lin_vel += self.vel_impulse_cache * UU_TO_BT;
            self.vel_impulse_cache = Vec3A::ZERO;
        }

        let vel = &mut rb.lin_vel;
        if vel.length_squared() > const { MAX_SPEED * MAX_SPEED } {
            *vel = vel.normalize_or_zero() * MAX_SPEED;
        }

        let ang_vel = &mut rb.ang_vel;
        if ang_vel.length_squared()
            > const { car_consts::MAX_ANG_SPEED * car_consts::MAX_ANG_SPEED }
        {
            *ang_vel = ang_vel.normalize_or_zero() * car_consts::MAX_ANG_SPEED;
        }

        self.state.phys.pos = rb.get_world_trans().translation * BT_TO_UU;
        self.state.phys.vel = rb.lin_vel * BT_TO_UU;
        self.state.phys.ang_vel = rb.ang_vel;
    }
}
