use glam::{Affine3A, Quat, Vec3A};

use crate::{
    CarBodyConfig, CarControls, CarState, MutatorConfig,
    bullet::{
        collision::BoxShape,
        dynamics::{
            discrete_dynamics_world::DiscreteDynamicsWorld,
            rigid_body::{Impulse, RigidBody},
            vehicle::{NUM_WHEELS, VehicleRL},
        },
        linear_math::QuatExt,
    },
    consts::{
        BT_TO_UU, UU_TO_BT, bullet_vehicle as vehicle_consts,
        car::{self as car_consts, drive as drive_consts},
        curves,
    },
};

#[derive(Clone, Copy, Debug)]
pub struct Car {
    pub bullet_vehicle: VehicleRL,
    pub state: CarState,
    sticky_gate_prev: bool,
}

impl Car {
    pub fn new(mutator_config: &MutatorConfig, config: CarBodyConfig) -> (Self, RigidBody) {
        let child_hitbox_shape = BoxShape::new(config.hitbox_size * UU_TO_BT * 0.5);
        let local_inertia = child_hitbox_shape.calculate_local_intertia(car_consts::MASS_BT);

        let body = RigidBody::new(Vec3A::ZERO, local_inertia, car_consts::MASS_BT);

        let mut bullet_vehicle = VehicleRL::default();

        for i in 0..NUM_WHEELS {
            let front = i < 2;
            let left = i % 2 == 0;

            let wheel_config = if front {
                &config.front_wheels
            } else {
                &config.back_wheels
            };

            let radius = wheel_config.wheel_radius;
            let suspension_rest_length =
                wheel_config.suspension_rest_length - vehicle_consts::MAX_SUSPENSION_TRAVEL;

            let wheel_ray_start_offset = if left {
                wheel_config.connection_point_offset * Vec3A::new(1.0, -1.0, 1.0)
            } else {
                wheel_config.connection_point_offset
            } * UU_TO_BT;
            bullet_vehicle.chassis_connection_point_cs[0][i] = wheel_ray_start_offset.x;
            bullet_vehicle.chassis_connection_point_cs[1][i] = wheel_ray_start_offset.y;
            bullet_vehicle.chassis_connection_point_cs[2][i] = wheel_ray_start_offset.z;

            bullet_vehicle.suspension_rest_length_1[i] = suspension_rest_length * UU_TO_BT;
            bullet_vehicle.wheel_radius[i] = radius * UU_TO_BT;
        }

        (
            Self {
                bullet_vehicle,
                state: CarState {
                    boost: mutator_config.car_spawn_boost_amount,
                    ..Default::default()
                },
                sticky_gate_prev: false,
            },
            body,
        )
    }

    pub const fn get_state(&self) -> &CarState {
        &self.state
    }

    pub const fn set_controls(&mut self, new_controls: CarControls) {
        self.state.controls = new_controls;
    }

    /// Reset transient wheel contacts for planning reuse. Keeps config.
    ///
    /// Drops cached contacts and the sticky gate so the next tick behaves
    /// like a fresh arena. Called via `Arena::reset_car_transient_contacts`;
    /// replay following must NOT call it.
    pub fn reset_transient_contacts(&mut self) {
        self.bullet_vehicle.reset_transient_contacts();
        self.sticky_gate_prev = false;
    }

    pub fn set_state(&mut self, rb: &mut RigidBody, state: &CarState) {
        rb.lin_vel = state.phys.vel * UU_TO_BT;
        rb.ang_vel = state.phys.ang_vel;
        rb.set_center_of_mass_trans(Affine3A {
            matrix3: state.phys.rot_mat,
            translation: state.phys.pos * UU_TO_BT,
        });
        rb.clear_accum_vels();

        // Hidden wheel contacts and the sticky gate carry over, matching live
        // replay following. Planning reuse must opt into freshness explicitly
        // via `Arena::reset_car_transient_contacts` (see `is_large_teleport`).
        self.state = *state;
    }

    fn update_wheels(
        &mut self,
        rb: &mut RigidBody,
        gravity: Vec3A,
        forward_speed_uu: f32,
        raw_throttle: f32,
        tick_time: f32,
    ) {
        let handbrake_delta = if self.state.controls.handbrake {
            drive_consts::POWERSLIDE_RISE_RATE
        } else {
            -drive_consts::POWERSLIDE_FALL_RATE
        } * tick_time;
        self.state.handbrake_val = (self.state.handbrake_val + handbrake_delta).clamp(0.0, 1.0);

        let mut real_brake = 0.0;

        let all_wheels_contact = self.state.wheels_with_contact.iter().all(|&w| w);
        let real_throttle =
            if self.state.controls.boost && self.state.boost > 0.0 && all_wheels_contact {
                1.0
            } else {
                raw_throttle
            };

        let abs_forward_speed_uu = forward_speed_uu.abs();
        let mut engine_throttle = real_throttle;
        if !self.state.controls.handbrake {
            if real_throttle.abs() >= drive_consts::THROTTLE_DEADZONE {
                if abs_forward_speed_uu > drive_consts::STOPPING_FORWARD_VEL
                    && real_throttle.signum() != forward_speed_uu.signum()
                {
                    real_brake = 1.0;

                    if abs_forward_speed_uu > drive_consts::BRAKING_NO_THROTTLE_SPEED_THRESH {
                        engine_throttle = 0.0;
                    }
                }
            } else if self.state.controls.boost && self.state.boost > 0.0 {
                engine_throttle = 1.0;
                real_brake = 0.0;
            } else {
                engine_throttle = 0.0;
                real_brake = if abs_forward_speed_uu < drive_consts::STOPPING_FORWARD_VEL {
                    1.0
                } else {
                    drive_consts::COASTING_BRAKE_FACTOR
                };
            }
        }

        let drive_speed_scale = curves::DRIVE_SPEED_TORQUE_FACTOR.get_output(abs_forward_speed_uu);
        self.bullet_vehicle.engine_force = engine_throttle
            * const { drive_consts::THROTTLE_TORQUE_AMOUNT * UU_TO_BT }
            * drive_speed_scale;
        self.bullet_vehicle.brake =
            real_brake * const { drive_consts::BRAKE_TORQUE_AMOUNT * UU_TO_BT };

        let mut steer_angle = curves::STEER_ANGLE_FROM_SPEED.get_output(abs_forward_speed_uu);
        if self.state.handbrake_val != 0.0 {
            steer_angle += (curves::POWERSLIDE_STEER_ANGLE_FROM_SPEED
                .get_output(abs_forward_speed_uu)
                - steer_angle)
                * self.state.handbrake_val;
        }

        steer_angle *= self.state.controls.steer;
        let steering_orn =
            Quat::from_axis_angle_simd(rb.get_world_trans().matrix3.z_axis, steer_angle);
        self.bullet_vehicle.steering_orn[0] = steering_orn;
        self.bullet_vehicle.steering_orn[1] = steering_orn;

        // fresh raycast contact must not produce sticky force within its own tick
        if self.sticky_gate_prev {
            const UPWARDS_DIR: Vec3A = Vec3A::Z;

            // Sticky keeps raw throttle
            let full_stick =
                raw_throttle != 0.0 || abs_forward_speed_uu > drive_consts::STOPPING_FORWARD_VEL;
            let mut sticky_force_scale = 0.5;
            if full_stick {
                sticky_force_scale += 1.0 - UPWARDS_DIR.z.abs();
            }

            rb.add_impulse(
                Impulse::Linear(UPWARDS_DIR * sticky_force_scale * gravity.z * tick_time),
                false,
                true,
            );
        }
    }

    fn update_boost(&mut self, rb: &mut RigidBody, mutator_config: &MutatorConfig, tick_time: f32) {
        let accel = mutator_config.boost_accel_ground;

        if self.state.is_boosting {
            let cost = mutator_config.boost_used_per_second * tick_time;
            self.state.boost = (self.state.boost - cost).max(0.0);
            let depleted = self.state.boost == 0.0;
            let latch_expired = !self.state.controls.boost
                && self.state.boosting_time >= car_consts::boost::MIN_TIME;
            if depleted || latch_expired {
                self.state.is_boosting = false;
                self.state.boosting_time = 0.0;
                self.state.time_since_boosted += tick_time;

                if mutator_config.recharge_boost_enabled
                    && self.state.time_since_boosted >= mutator_config.recharge_boost_delay
                {
                    self.state.boost += mutator_config.recharge_boost_per_second * tick_time;
                }
            } else {
                self.state.is_boosting = true;
                self.state.boosting_time += tick_time;
                self.state.time_since_boosted = 0.0;

                rb.add_impulse(
                    Impulse::Linear(self.state.get_forward_dir() * (accel * UU_TO_BT) * tick_time),
                    false,
                    true,
                );
            }
        } else if self.state.controls.boost && self.state.boost > 0.0 {
            self.state.is_boosting = true;
            self.state.boosting_time += tick_time;
            self.state.time_since_boosted = 0.0;

            rb.add_impulse(
                Impulse::Linear(self.state.get_forward_dir() * (accel * UU_TO_BT) * tick_time),
                false,
                true,
            );
        } else {
            self.state.is_boosting = false;
            self.state.boosting_time = 0.0;
            self.state.time_since_boosted += tick_time;

            if mutator_config.recharge_boost_enabled
                && self.state.time_since_boosted >= mutator_config.recharge_boost_delay
            {
                self.state.boost += mutator_config.recharge_boost_per_second * tick_time;
            }
        }

        self.state.boost = self.state.boost.clamp(0.0, car_consts::boost::MAX);
    }

    pub fn pre_tick_update(
        &mut self,
        collision_world: &mut DiscreteDynamicsWorld,
        mutator_config: &MutatorConfig,
        tick_time: f32,
    ) {
        self.state.controls = self.state.controls.clamp();
        let forward_speed_uu = collision_world.collision_obj.get_forward_speed() * BT_TO_UU;

        let raw_throttle = self.state.controls.throttle;

        self.update_wheels(
            &mut collision_world.collision_obj,
            mutator_config.gravity * UU_TO_BT,
            forward_speed_uu,
            raw_throttle,
            tick_time,
        );

        self.update_boost(
            &mut collision_world.collision_obj,
            mutator_config,
            tick_time,
        );

        let real_throttle_vehicle = if self.state.controls.boost && self.state.boost > 0.0 {
            1.0
        } else {
            raw_throttle
        };
        self.bullet_vehicle.update(
            &mut collision_world.collision_obj,
            tick_time,
            self.state.handbrake_val,
            real_throttle_vehicle,
        );
        let in_contact = self.bullet_vehicle.had_world_contact;
        self.state.wheels_with_contact = [in_contact; 4];
        self.sticky_gate_prev = in_contact;
    }

    pub fn finish_physics_tick(&mut self, rb: &mut RigidBody) {
        self.state.phys.rot_mat = rb.get_world_trans().matrix3;
        self.state.phys.pos = rb.get_world_trans().translation * BT_TO_UU;
        self.state.phys.vel = rb.lin_vel * BT_TO_UU;
        self.state.phys.ang_vel = rb.ang_vel;
    }
}
