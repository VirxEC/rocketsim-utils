use glam::{Affine3A, Quat, Vec3A, Vec4};

use crate::{
    CarBodyConfig, CarControls, CarState, MutatorConfig,
    bullet::{
        collision::BoxShape,
        dynamics::{
            discrete_dynamics_world::DiscreteDynamicsWorld,
            rigid_body::RigidBody,
            vehicle::{
                vehicle_rl::{NUM_WHEELS, VehicleRL},
                wheel_info::WheelInfo,
            },
        },
        linear_math::{QuatExt, to_simd},
    },
    consts::{
        self, BT_TO_UU, TICK_TIME, UU_TO_BT, bullet_vehicle as vehicle_consts,
        car::{self as car_consts, drive as drive_consts},
        curves,
    },
};

pub struct Car {
    pub bullet_vehicle: VehicleRL,
    pub state: CarState,
}

impl Car {
    pub fn new(mutator_config: &MutatorConfig, config: CarBodyConfig) -> (Self, RigidBody) {
        let child_hitbox_shape = BoxShape::new(config.hitbox_size * UU_TO_BT * 0.5);
        let local_inertia = child_hitbox_shape.calculate_local_intertia(car_consts::MASS_BT);

        let body = RigidBody::new(Vec3A::ZERO, local_inertia, car_consts::MASS_BT);

        let mut wheels = [WheelInfo::default(); NUM_WHEELS];

        for (i, wheel) in wheels.iter_mut().enumerate() {
            let front = i < 2;
            let left = i % 2 != 0;

            let (wheel_config, suspension_force_scale) = if front {
                (
                    &config.front_wheels,
                    vehicle_consts::SUSPENSION_FORCE_SCALE_FRONT,
                )
            } else {
                (
                    &config.back_wheels,
                    vehicle_consts::SUSPENSION_FORCE_SCALE_BACK,
                )
            };

            let radius = wheel_config.wheel_radius;
            let suspension_rest_length =
                wheel_config.suspension_rest_length - vehicle_consts::MAX_SUSPENSION_TRAVEL;

            let wheel_ray_start_offset = if left {
                wheel_config.connection_point_offset * Vec3A::new(1.0, -1.0, 1.0)
            } else {
                wheel_config.connection_point_offset
            };

            *wheel = WheelInfo::new(
                wheel_ray_start_offset * UU_TO_BT,
                suspension_rest_length * UU_TO_BT,
                radius * UU_TO_BT,
                suspension_force_scale,
            );

            wheel.update_wheel_trans::<true>(body.get_world_trans());
        }

        (
            Self {
                bullet_vehicle: VehicleRL::new(wheels),
                state: CarState {
                    boost: mutator_config.car_spawn_boost_amount,
                    ..Default::default()
                },
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

    pub fn set_state(&mut self, rb: &mut RigidBody, state: &CarState) {
        rb.lin_vel = state.phys.vel * UU_TO_BT;
        rb.ang_vel = state.phys.ang_vel;
        rb.set_center_of_mass_trans(Affine3A {
            matrix3: state.phys.rot_mat,
            translation: state.phys.pos * UU_TO_BT,
        });

        self.state = *state;
    }

    fn update_wheels(&mut self, rb: &mut RigidBody, forward_speed_uu: f32) {
        const STICKY_FORCE_SCALE: Vec3A = Vec3A::new(0.0, 0.0, 0.5 * consts::GRAVITY_Z * UU_TO_BT);

        let handbrake_delta = if self.state.controls.handbrake {
            drive_consts::POWERSLIDE_RISE_RATE
        } else {
            -drive_consts::POWERSLIDE_FALL_RATE
        } * TICK_TIME;
        self.state.handbrake_val = (self.state.handbrake_val + handbrake_delta).clamp(0.0, 1.0);

        let mut real_brake = 0.0;
        let real_throttle = if self.state.controls.boost && self.state.boost > 0.0 {
            1.0
        } else {
            self.state.controls.throttle
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

        let drive_engine_force = engine_throttle
            * const { drive_consts::THROTTLE_TORQUE_AMOUNT * UU_TO_BT }
            * drive_speed_scale;
        let drive_brake_force = real_brake * const { drive_consts::BRAKE_TORQUE_AMOUNT * UU_TO_BT };
        self.bullet_vehicle.engine_force = drive_engine_force;
        self.bullet_vehicle.brake = drive_brake_force;

        let mut steer_angle = curves::STEER_ANGLE_FROM_SPEED.get_output(abs_forward_speed_uu);
        if self.state.handbrake_val != 0.0 {
            steer_angle += (curves::POWERSLIDE_STEER_ANGLE_FROM_SPEED
                .get_output(abs_forward_speed_uu)
                - steer_angle)
                * self.state.handbrake_val;
        }

        steer_angle *= self.state.controls.steer;
        let steering_orn = Quat::from_angle_axis_up(steer_angle);
        self.bullet_vehicle.wheels[0].steering_orn = steering_orn;
        self.bullet_vehicle.wheels[1].steering_orn = steering_orn;

        let car_pos = rb.get_world_pos();
        let car_vel = rb.lin_vel;
        let car_ang_vel = rb.ang_vel;

        let mut lat_dir_x = [0.0; NUM_WHEELS];
        let mut lat_dir_y = [0.0; NUM_WHEELS];
        let mut lat_dir_z = [0.0; NUM_WHEELS];
        let mut wheel_delta_x = [0.0; NUM_WHEELS];
        let mut wheel_delta_y = [0.0; NUM_WHEELS];
        let mut wheel_delta_z = [0.0; NUM_WHEELS];

        for (i, wheel) in self.bullet_vehicle.wheels.iter().enumerate() {
            let lat_dir = wheel.axle_dir;
            let wheel_delta = wheel.raycast_info.hard_point_ws - car_pos;

            lat_dir_x[i] = lat_dir.x;
            lat_dir_y[i] = lat_dir.y;
            lat_dir_z[i] = lat_dir.z;

            wheel_delta_x[i] = wheel_delta.x;
            wheel_delta_y[i] = wheel_delta.y;
            wheel_delta_z[i] = wheel_delta.z;
        }

        let [lat_dir_x, lat_dir_y, lat_dir_z] = to_simd(&[lat_dir_x, lat_dir_y, lat_dir_z]);
        let [wheel_delta_x, wheel_delta_y, wheel_delta_z] =
            to_simd(&[wheel_delta_x, wheel_delta_y, wheel_delta_z]);

        let avx = Vec4::splat(car_ang_vel.x);
        let avy = Vec4::splat(car_ang_vel.y);
        let avz = Vec4::splat(car_ang_vel.z);

        let cross_x = avy * wheel_delta_z - avz * wheel_delta_y;
        let cross_y = avz * wheel_delta_x - avx * wheel_delta_z;
        let cross_z = avx * wheel_delta_y - avy * wheel_delta_x;

        let cross_vec_x = (cross_x + car_vel.x) * BT_TO_UU;
        let cross_vec_y = (cross_y + car_vel.y) * BT_TO_UU;
        let cross_vec_z = (cross_z + car_vel.z) * BT_TO_UU;

        let base_friction =
            (cross_vec_x * lat_dir_x + cross_vec_y * lat_dir_y + cross_vec_z * lat_dir_z).abs();
        let long_dot = (cross_vec_x * lat_dir_y - cross_vec_y * lat_dir_x).abs();

        for i in 0..NUM_WHEELS {
            let base_friction = base_friction[i];
            let friction_curve_input = if base_friction > 5.0 {
                base_friction / (long_dot[i] + base_friction)
            } else {
                0.0
            };

            let mut lat_friction = curves::LAT_FRICTION.get_output(friction_curve_input);
            let mut long_friction = 1.0;

            if self.state.handbrake_val != 0.0 {
                lat_friction *=
                    1.0 - curves::HANDBRAKE_LAT_FRICTION_FACTOR * self.state.handbrake_val;
                long_friction *= 1.0
                    + (curves::HANDBRAKE_LONG_FRICTION_FACTOR.get_output(friction_curve_input)
                        - 1.0)
                        * self.state.handbrake_val;
            }

            self.bullet_vehicle.lat_friction[i] = lat_friction;
            self.bullet_vehicle.long_friction[i] = long_friction;
        }

        rb.apply_central_force(STICKY_FORCE_SCALE);
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
                self.state.get_forward_dir() * (consts::car::boost::ACCEL_GROUND * UU_TO_BT),
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

    pub fn pre_tick_update(
        &mut self,
        collision_world: &mut DiscreteDynamicsWorld,
        mutator_config: &MutatorConfig,
    ) {
        self.state.controls = self.state.controls.clamp();
        let forward_speed_uu = collision_world.collision_obj.get_forward_speed() * BT_TO_UU;

        // Do first part of the btVehicleRL update (update wheel transforms, do traces, calculate friction impulses)
        self.bullet_vehicle.update_vehicle_first(collision_world);

        self.update_wheels(&mut collision_world.collision_obj, forward_speed_uu);

        self.bullet_vehicle
            .update_vehicle_second(&mut collision_world.collision_obj, TICK_TIME);
        self.update_boost(&mut collision_world.collision_obj, mutator_config);
    }

    pub fn finish_physics_tick(&mut self, rb: &mut RigidBody) {
        const MAX_SPEED: f32 = car_consts::MAX_SPEED * UU_TO_BT;

        let lin_vel_sq = rb.lin_vel.length_squared();
        if lin_vel_sq > MAX_SPEED * MAX_SPEED {
            rb.lin_vel = rb.lin_vel / lin_vel_sq.sqrt() * MAX_SPEED;
        }

        let ang_vel_sq = rb.ang_vel.length_squared();
        if ang_vel_sq > car_consts::MAX_ANG_SPEED * car_consts::MAX_ANG_SPEED {
            rb.ang_vel = rb.ang_vel / ang_vel_sq.sqrt() * car_consts::MAX_ANG_SPEED;
        }

        self.state.phys.rot_mat = rb.get_world_trans().matrix3;
        self.state.phys.pos = rb.get_world_trans().translation * BT_TO_UU;
        self.state.phys.vel = rb.lin_vel * BT_TO_UU;
        self.state.phys.ang_vel = rb.ang_vel;
    }
}
