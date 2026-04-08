use glam::{Vec3A, Vec4};

use super::wheel_info::WheelInfo;
use crate::{
    bullet::{
        collision::StaticPlaneShape,
        dynamics::{contact_constraint::resolve_single_bilateral, rigid_body::RigidBody},
        linear_math::{from_simd, to_simd},
    },
    consts::{UU_TO_BT, bullet_vehicle},
};

pub const NUM_WHEELS: usize = 4;
const SUSPENSION_TRAVEL: f32 = bullet_vehicle::MAX_SUSPENSION_TRAVEL * UU_TO_BT;

#[derive(Default)]
struct RaycastInfo {
    contact_point_ws: [Vec4; 3],
    hard_point_ws: [Vec4; 3],
    wheel_direction_ws: Vec3A,
}

#[derive(Default)]
pub struct VehicleRL {
    pub wheels: [WheelInfo; NUM_WHEELS],
    raycast_info: RaycastInfo,
    pub chassis_connection_point_cs: [Vec4; 3],
    pub engine_force: f32,
    pub brake: f32,
    pub suspension_force_scale: Vec4,
    pub lat_friction: [f32; 4],
    pub long_friction: [f32; 4],
    impulse: [Vec4; 3],
}

impl VehicleRL {
    fn calc_friction_impulses(&mut self, chassis: &RigidBody, friction_scale: f32) {
        const ROLLING_FRICTION_SCALE: f32 = 113.73963;

        let mut axle_x = [0.0; 4];
        let mut axle_y = [0.0; 4];
        let mut axle_z = [0.0; 4];

        for (i, wheel) in self.wheels.iter().enumerate() {
            let axle_dir = wheel.axle_dir;
            axle_x[i] = axle_dir.x;
            axle_y[i] = axle_dir.y;
            axle_z[i] = axle_dir.z;
        }

        let [axle_x, axle_y, axle_z] = to_simd(&[axle_x, axle_y, axle_z]);

        let chassis_pos = chassis.get_world_trans().translation;
        let rel_x = self.raycast_info.contact_point_ws[0] - chassis_pos.x;
        let rel_y = self.raycast_info.contact_point_ws[1] - chassis_pos.y;
        let rel_z = self.raycast_info.contact_point_ws[2] - chassis_pos.z;

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
            resolve_single_bilateral(chassis, rel_x, rel_y, rel_z, axle_x, axle_y, axle_z);

        let rolling_friction = if self.engine_force == 0.0 {
            let brake = Vec4::splat(self.brake);
            (rel_vel * ROLLING_FRICTION_SCALE).clamp(-brake, brake)
        } else {
            Vec4::splat(self.engine_force / friction_scale)
        };

        let rf = -rolling_friction * Vec4::from_array(self.long_friction);
        let si = side_impulse * Vec4::from_array(self.lat_friction);

        let total_x = axle_x * si - axle_y * rf;
        let total_y = axle_x * rf + axle_y * si;
        let total_z = axle_z * rf + axle_z * si;

        let scale = Vec4::splat(friction_scale);
        self.impulse[0] = total_x * scale;
        self.impulse[1] = total_y * scale;
        self.impulse[2] = total_z * scale;
    }

    fn apply_friction_impulses(&self, cb: &mut RigidBody, time_step: f32) {
        let trans = cb.get_world_trans();
        let rel_x = self.raycast_info.contact_point_ws[0] - trans.translation.x;
        let rel_y = self.raycast_info.contact_point_ws[1] - trans.translation.y;
        let rel_z = self.raycast_info.contact_point_ws[2] - trans.translation.z;

        let up_x = Vec4::splat(trans.matrix3.z_axis.x);
        let up_y = Vec4::splat(trans.matrix3.z_axis.y);
        let up_z = Vec4::splat(trans.matrix3.z_axis.z);

        let contact_up_dot = up_x * rel_x + up_y * rel_y + up_z * rel_z;

        let wheel_rel_x = rel_x - up_x * contact_up_dot;
        let wheel_rel_y = rel_y - up_y * contact_up_dot;
        let wheel_rel_z = rel_z - up_z * contact_up_dot;

        let scale = Vec4::splat(time_step);
        let impulse_x = self.impulse[0] * scale;
        let impulse_y = self.impulse[1] * scale;
        let impulse_z = self.impulse[2] * scale;

        cb.apply_impulses(
            impulse_x,
            impulse_y,
            impulse_z,
            wheel_rel_x,
            wheel_rel_y,
            wheel_rel_z,
        );
    }

    fn get_raycast_targets(&self) -> [Vec4; 3] {
        let mut rest_len = [0.0; 4];
        let mut wheel_radius = [0.0; 4];

        for (i, wheel) in self.wheels.iter().enumerate() {
            rest_len[i] = wheel.suspension_rest_length_1;
            wheel_radius[i] = wheel.wheel_radius;
        }

        let ray_length = Vec4::from_array(rest_len)
            + Vec4::from_array(wheel_radius)
            + (SUSPENSION_TRAVEL - bullet_vehicle::SUSPENSION_SUBTRACTION);

        let target_x = self.raycast_info.hard_point_ws[0]
            + self.raycast_info.wheel_direction_ws.x * ray_length;
        let target_y = self.raycast_info.hard_point_ws[1]
            + self.raycast_info.wheel_direction_ws.y * ray_length;
        let target_z = self.raycast_info.hard_point_ws[2]
            + self.raycast_info.wheel_direction_ws.z * ray_length;

        [target_x, target_y, target_z]
    }

    pub fn update_vehicle_first(&mut self, chassis: &RigidBody) {
        let chassis_trans = chassis.get_world_trans();
        let basis_x = chassis_trans.matrix3.x_axis;
        let basis_y = chassis_trans.matrix3.y_axis;
        let basis_z = chassis_trans.matrix3.z_axis;

        let hard_x = self.chassis_connection_point_cs[0] * basis_x.x
            + self.chassis_connection_point_cs[1] * basis_y.x
            + self.chassis_connection_point_cs[2] * basis_z.x
            + chassis_trans.translation.x;
        let hard_y = self.chassis_connection_point_cs[0] * basis_x.y
            + self.chassis_connection_point_cs[1] * basis_y.y
            + self.chassis_connection_point_cs[2] * basis_z.y
            + chassis_trans.translation.y;
        let hard_z = self.chassis_connection_point_cs[0] * basis_x.z
            + self.chassis_connection_point_cs[1] * basis_y.z
            + self.chassis_connection_point_cs[2] * basis_z.z
            + chassis_trans.translation.z;

        self.raycast_info.hard_point_ws = [hard_x, hard_y, hard_z];
        self.raycast_info.wheel_direction_ws = -basis_z;

        for (i, wheel) in self.wheels.iter_mut().enumerate() {
            wheel.raycast_info.hard_point_ws = Vec3A::new(hard_x[i], hard_y[i], hard_z[i]);
        }

        for wheel in &mut self.wheels[0..2] {
            wheel.axle_dir = wheel.steering_orn * chassis_trans.matrix3.y_axis;
        }

        for wheel in &mut self.wheels[2..] {
            wheel.axle_dir = chassis_trans.matrix3.y_axis;
        }

        let targets = self.get_raycast_targets();

        let ray_results =
            StaticPlaneShape::perform_raycast(&self.raycast_info.hard_point_ws, &targets);
        self.raycast_info.contact_point_ws = ray_results;

        let ray_results = from_simd(&ray_results);
        for (i, wheel) in self.wheels.iter_mut().enumerate() {
            wheel.apply_ray_cast(chassis, ray_results[i]);
        }

        self.calc_friction_impulses(chassis, chassis.mass / 3.0);
    }

    fn update_suspension(&mut self, cb: &mut RigidBody, delta_time: f32) {
        const COMPRESSION_DAMPING: Vec4 = Vec4::splat(bullet_vehicle::WHEELS_DAMPING_COMPRESSION);
        const RELAXATION_DAMPING: Vec4 = Vec4::splat(bullet_vehicle::WHEELS_DAMPING_RELAXATION);

        let mut rest_len = [0.0; 4];
        let mut susp_len = [0.0; 4];
        let mut rel_vel = [0.0; 4];

        for (i, wheel) in self.wheels.iter().enumerate() {
            rest_len[i] = wheel.suspension_rest_length_1;
            susp_len[i] = wheel.suspension_length;
            rel_vel[i] = wheel.suspension_relative_vel;
        }

        let [rest_len, susp_len, rel_vel] = to_simd(&[rest_len, susp_len, rel_vel]);

        let force = (rest_len - susp_len) * bullet_vehicle::SUSPENSION_STIFFNESS;

        let damping_vel_scale = Vec4::select(
            rel_vel.cmplt(Vec4::ZERO),
            COMPRESSION_DAMPING,
            RELAXATION_DAMPING,
        );

        let suspension_force = (force - damping_vel_scale * rel_vel) * self.suspension_force_scale;
        let suspension_force = suspension_force.max(Vec4::ZERO) * delta_time;

        let trans = cb.get_world_trans();
        let rel_x = self.raycast_info.contact_point_ws[0] - trans.translation.x;
        let rel_y = self.raycast_info.contact_point_ws[1] - trans.translation.y;

        let total_force =
            suspension_force.x + suspension_force.y + suspension_force.z + suspension_force.w;
        cb.lin_vel.z += total_force * cb.inverse_mass;

        let torque_x = rel_y * suspension_force;
        let torque_y = -rel_x * suspension_force;

        cb.ang_vel += cb.inv_inertia_tensor_world.x_axis * torque_x.element_sum()
            + cb.inv_inertia_tensor_world.y_axis * torque_y.element_sum();
    }

    pub fn update_vehicle_second(&mut self, cb: &mut RigidBody, step: f32) {
        self.update_suspension(cb, step);
        self.apply_friction_impulses(cb, step);
    }
}
