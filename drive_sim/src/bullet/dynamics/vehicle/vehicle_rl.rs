use glam::{Vec3A, Vec4};

use super::wheel_info::WheelInfo;
use crate::{
    bullet::{
        collision::StaticPlaneShape,
        dynamics::{
            contact_constraint::resolve_single_bilateral,
            discrete_dynamics_world::DiscreteDynamicsWorld, rigid_body::RigidBody,
        },
        linear_math::{from_simd, to_simd},
    },
    consts::{UU_TO_BT, bullet_vehicle},
};

pub const NUM_WHEELS: usize = 4;
const SUSPENSION_TRAVEL: f32 = bullet_vehicle::MAX_SUSPENSION_TRAVEL * UU_TO_BT;

pub struct VehicleRL {
    pub wheels: [WheelInfo; NUM_WHEELS],
    pub engine_force: f32,
    pub brake: f32,
    pub lat_friction: [f32; 4],
    pub long_friction: [f32; 4],
    pub impulse: [Vec4; 3],
    pub contact_point_ws: [Vec4; 3],
}

impl VehicleRL {
    pub fn new(wheels: [WheelInfo; NUM_WHEELS]) -> Self {
        Self {
            wheels,
            engine_force: 0.0,
            brake: 0.0,
            lat_friction: [1.0; 4],
            long_friction: [1.0; 4],
            impulse: [Vec4::ZERO; 3],
            contact_point_ws: [Vec4::ZERO; 3],
        }
    }

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
        let rel_x = self.contact_point_ws[0] - chassis_pos.x;
        let rel_y = self.contact_point_ws[1] - chassis_pos.y;
        let rel_z = self.contact_point_ws[2] - chassis_pos.z;

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
        let rel_x = self.contact_point_ws[0] - trans.translation.x;
        let rel_y = self.contact_point_ws[1] - trans.translation.y;
        let rel_z = self.contact_point_ws[2] - trans.translation.z;

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

        for i in 0..4 {
            let impulse = Vec3A::new(impulse_x[i], impulse_y[i], impulse_z[i]);
            let rel_pos = Vec3A::new(wheel_rel_x[i], wheel_rel_y[i], wheel_rel_z[i]);
            cb.apply_impulse(impulse, rel_pos);
        }
    }

    fn get_raycast_info_simd(&self) -> ([Vec4; 3], [Vec4; 3]) {
        let mut source_x = [0.0; 4];
        let mut source_y = [0.0; 4];
        let mut source_z = [0.0; 4];
        let mut dir_x = [0.0; 4];
        let mut dir_y = [0.0; 4];
        let mut dir_z = [0.0; 4];
        let mut rest_len = [0.0; 4];
        let mut wheel_radius = [0.0; 4];

        for (i, wheel) in self.wheels.iter().enumerate() {
            let source = wheel.raycast_info.hard_point_ws;
            source_x[i] = source.x;
            source_y[i] = source.y;
            source_z[i] = source.z;

            let dir = wheel.raycast_info.wheel_direction_ws;
            dir_x[i] = dir.x;
            dir_y[i] = dir.y;
            dir_z[i] = dir.z;

            rest_len[i] = wheel.suspension_rest_length_1;
            wheel_radius[i] = wheel.wheel_radius;
        }

        let [source_x, source_y, source_z] = to_simd(&[source_x, source_y, source_z]);
        let [dir_x, dir_y, dir_z] = to_simd(&[dir_x, dir_y, dir_z]);

        let ray_length = Vec4::from_array(rest_len)
            + Vec4::from_array(wheel_radius)
            + (SUSPENSION_TRAVEL - bullet_vehicle::SUSPENSION_SUBTRACTION);

        let target_x = source_x + dir_x * ray_length;
        let target_y = source_y + dir_y * ray_length;
        let target_z = source_z + dir_z * ray_length;

        (
            [source_x, source_y, source_z],
            [target_x, target_y, target_z],
        )
    }

    pub fn update_vehicle_first(&mut self, collision_world: &DiscreteDynamicsWorld) {
        let chassis = &collision_world.collision_obj;

        for wheel in &mut self.wheels[0..2] {
            wheel.update_wheel_trans::<true>(chassis.get_world_trans());
        }

        for wheel in &mut self.wheels[2..] {
            wheel.update_wheel_trans::<false>(chassis.get_world_trans());
        }

        let (sources, targets) = self.get_raycast_info_simd();

        let ray_results = StaticPlaneShape::perform_raycast(&sources, &targets);
        self.contact_point_ws = ray_results;

        let ray_results = from_simd(&ray_results);
        for (i, wheel) in self.wheels.iter_mut().enumerate() {
            wheel.apply_ray_cast(chassis, ray_results[i]);
        }

        self.calc_friction_impulses(chassis, chassis.mass / 3.0);
    }

    pub fn update_vehicle_second(&mut self, cb: &mut RigidBody, step: f32) {
        for wheel in &mut self.wheels {
            wheel.update_suspension(cb, step);
        }

        self.apply_friction_impulses(cb, step);
    }
}
