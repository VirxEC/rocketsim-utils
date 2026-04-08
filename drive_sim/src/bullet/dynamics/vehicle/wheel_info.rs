use glam::{Quat, Vec3A};

use crate::{
    bullet::dynamics::rigid_body::RigidBody,
    consts::{UU_TO_BT, bullet_vehicle},
};

const SUSPENSION_TRAVEL: f32 = bullet_vehicle::MAX_SUSPENSION_TRAVEL * UU_TO_BT;

#[derive(Clone, Copy, Default)]
pub struct RaycastInfo {
    pub contact_point_ws: Vec3A,
    pub hard_point_ws: Vec3A,
}

#[derive(Clone, Copy, Default)]
pub struct WheelInfo {
    pub suspension_length: f32,
    pub suspension_rest_length_1: f32,
    pub wheel_radius: f32,
    pub suspension_relative_vel: f32,
    pub raycast_info: RaycastInfo,
    pub axle_dir: Vec3A,
    pub steering_orn: Quat,
}

impl WheelInfo {
    pub fn new(suspension_rest_length_1: f32, wheel_radius: f32) -> Self {
        Self {
            suspension_rest_length_1,
            wheel_radius,
            suspension_length: 0.0,
            raycast_info: RaycastInfo::default(),
            axle_dir: Vec3A::ZERO,
            suspension_relative_vel: 0.0,
            steering_orn: Quat::IDENTITY,
        }
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
}
