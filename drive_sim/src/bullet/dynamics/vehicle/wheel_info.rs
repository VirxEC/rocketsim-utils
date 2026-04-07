use glam::{Affine3A, Quat, Vec3A, Vec4};

use crate::{
    bullet::dynamics::{contact_constraint::resolve_single_bilateral_simd, rigid_body::RigidBody},
    consts::{UU_TO_BT, bullet_vehicle},
};

const SUSPENSION_TRAVEL: f32 = bullet_vehicle::MAX_SUSPENSION_TRAVEL * UU_TO_BT;

#[derive(Clone, Copy, Default)]
pub struct RaycastInfo {
    contact_point_ws: Vec3A,
    pub hard_point_ws: Vec3A,
    wheel_direction_ws: Vec3A,
}

#[derive(Clone, Copy, Default)]
pub struct WheelInfo {
    chassis_connection_point_cs: Vec3A,
    suspension_length: f32,
    suspension_rest_length_1: f32,
    suspension_force_scale: f32,
    wheel_radius: f32,
    suspension_relative_vel: f32,
    wheels_suspension_force: f32,
    impulse: Vec3A,
    pub raycast_info: RaycastInfo,
    pub axle_dir: Vec3A,
    pub engine_force: f32,
    pub brake: f32,
    pub steering_orn: Quat,
    pub lat_friction: f32,
    pub long_friction: f32,
}

impl WheelInfo {
    pub fn new(
        chassis_connection_point_cs: Vec3A,
        suspension_rest_length_1: f32,
        wheel_radius: f32,
        suspension_force_scale: f32,
    ) -> Self {
        Self {
            suspension_rest_length_1,
            wheel_radius,
            chassis_connection_point_cs,
            suspension_force_scale,
            suspension_length: 0.0,
            engine_force: 0.0,
            brake: 0.0,
            raycast_info: RaycastInfo::default(),
            axle_dir: Vec3A::ZERO,
            suspension_relative_vel: 0.0,
            wheels_suspension_force: 0.0,
            steering_orn: Quat::IDENTITY,
            lat_friction: 0.0,
            long_friction: 0.0,
            impulse: Vec3A::ZERO,
        }
    }

    pub fn update_wheel_trans<const FRONT: bool>(&mut self, chassis_trans: &Affine3A) {
        self.raycast_info.hard_point_ws =
            chassis_trans.transform_point3a(self.chassis_connection_point_cs);
        self.raycast_info.wheel_direction_ws = -chassis_trans.matrix3.z_axis;
        self.axle_dir = if FRONT {
            self.steering_orn * chassis_trans.matrix3.y_axis
        } else {
            chassis_trans.matrix3.y_axis
        };
    }

    pub fn get_raycast_info(&self) -> (Vec3A, Vec3A) {
        let real_ray_length = self.suspension_rest_length_1 + self.wheel_radius + SUSPENSION_TRAVEL
            - bullet_vehicle::SUSPENSION_SUBTRACTION;

        let source = self.raycast_info.hard_point_ws;
        let target = source + self.raycast_info.wheel_direction_ws * real_ray_length;

        (source, target)
    }

    pub fn apply_ray_cast(&mut self, chassis: &RigidBody, hit_point_in_world: Vec3A) {
        self.raycast_info.contact_point_ws = hit_point_in_world;

        let wheel_trace_len_sq =
            self.raycast_info.hard_point_ws.z - self.raycast_info.contact_point_ws.z;
        self.suspension_length = wheel_trace_len_sq - self.wheel_radius;

        let min_suspension_len = self.suspension_rest_length_1 - SUSPENSION_TRAVEL;
        let max_suspension_len = self.suspension_rest_length_1 + SUSPENSION_TRAVEL;
        self.suspension_length = self
            .suspension_length
            .clamp(min_suspension_len, max_suspension_len);

        let rel_pos = self.raycast_info.contact_point_ws - chassis.get_world_trans().translation;

        let vel_at_contact_point = chassis.get_vel_in_local_point(rel_pos);
        self.suspension_relative_vel = vel_at_contact_point.z;
    }

    pub fn calc_friction_impulses_simd(
        wheels: &mut [WheelInfo; 4],
        chassis: &RigidBody,
        friction_scale: f32,
    ) {
        const ROLLING_FRICTION_SCALE: f32 = 113.73963;

        let mut axle_x = Vec4::ZERO;
        let mut axle_y = Vec4::ZERO;
        let mut axle_z = Vec4::ZERO;
        let mut contact_x = Vec4::ZERO;
        let mut contact_y = Vec4::ZERO;
        let mut contact_z = Vec4::ZERO;
        let mut engine_force = Vec4::ZERO;
        let mut brake = Vec4::ZERO;
        let mut lat_friction = Vec4::ZERO;
        let mut long_friction = Vec4::ZERO;

        for (i, wheel) in wheels.iter().enumerate() {
            let axle_dir = wheel.axle_dir;
            axle_x[i] = axle_dir.x;
            axle_y[i] = axle_dir.y;
            axle_z[i] = axle_dir.z;

            let contact = wheel.raycast_info.contact_point_ws;
            contact_x[i] = contact.x;
            contact_y[i] = contact.y;
            contact_z[i] = contact.z;

            engine_force[i] = wheel.engine_force;
            brake[i] = wheel.brake;
            lat_friction[i] = wheel.lat_friction;
            long_friction[i] = wheel.long_friction;
        }

        let chassis_pos = chassis.get_world_trans().translation;
        let rel_x = contact_x - chassis_pos.x;
        let rel_y = contact_y - chassis_pos.y;
        let rel_z = contact_z - chassis_pos.z;

        let avx = Vec4::splat(chassis.ang_vel.x);
        let avy = Vec4::splat(chassis.ang_vel.y);
        let avz = Vec4::splat(chassis.ang_vel.z);

        let cross_x = avy * rel_z - avz * rel_y;
        let cross_y = avz * rel_x - avx * rel_z;
        let cross_z = avx * rel_y - avy * rel_x;

        let contact_vel_x = cross_x + chassis.lin_vel.x;
        let contact_vel_y = cross_y + chassis.lin_vel.y;
        let contact_vel_z = cross_z + chassis.lin_vel.z;

        let rel_vel = contact_vel_y * axle_x - contact_vel_x * axle_y + contact_vel_z * axle_z;

        let side_impulse =
            resolve_single_bilateral_simd(chassis, rel_x, rel_y, rel_z, axle_x, axle_y, axle_z);

        let braking_friction = (rel_vel * ROLLING_FRICTION_SCALE).clamp(-brake, brake);
        let engine_friction = engine_force / friction_scale;

        let rolling_friction = -Vec4::select(
            engine_force.cmpeq(Vec4::ZERO),
            braking_friction,
            engine_friction,
        );

        let rf = rolling_friction * long_friction;
        let si = side_impulse * lat_friction;

        let total_x = axle_x * si - axle_y * rf;
        let total_y = axle_x * rf + axle_y * si;
        let total_z = axle_z * rf + axle_z * si;

        let scale = Vec4::splat(friction_scale);
        let impulse_x = total_x * scale;
        let impulse_y = total_y * scale;
        let impulse_z = total_z * scale;

        for i in 0..4 {
            wheels[i].impulse = Vec3A::new(impulse_x[i], impulse_y[i], impulse_z[i]);
        }
    }

    pub fn update_suspension(&mut self, cb: &mut RigidBody, delta_time: f32) {
        let force = (self.suspension_rest_length_1 - self.suspension_length)
            * bullet_vehicle::SUSPENSION_STIFFNESS;

        let damping_vel_scale = if self.suspension_relative_vel < 0.0 {
            bullet_vehicle::WHEELS_DAMPING_COMPRESSION
        } else {
            bullet_vehicle::WHEELS_DAMPING_RELAXATION
        };

        self.wheels_suspension_force = force - (damping_vel_scale * self.suspension_relative_vel);

        self.wheels_suspension_force *= self.suspension_force_scale;
        self.wheels_suspension_force = self.wheels_suspension_force.max(0.0);

        if self.wheels_suspension_force == 0.0 {
            return;
        }

        let base_force_scale = self.wheels_suspension_force * delta_time;
        let contact_point_offset =
            self.raycast_info.contact_point_ws - cb.get_world_trans().translation;

        let force = Vec3A::new(0.0, 0.0, base_force_scale);
        cb.apply_impulse(force, contact_point_offset);
    }

    pub fn apply_friction_impulses(&self, cb: &mut RigidBody, time_step: f32) {
        if self.impulse == Vec3A::ZERO {
            return;
        }

        let trans = cb.get_world_trans();
        let wheel_contact_offset = self.raycast_info.contact_point_ws - trans.translation;
        let contact_up_dot = trans.matrix3.z_axis.dot(wheel_contact_offset);
        let wheel_rel_pos = wheel_contact_offset - trans.matrix3.z_axis * contact_up_dot;
        cb.apply_impulse(self.impulse * time_step, wheel_rel_pos);
    }
}
