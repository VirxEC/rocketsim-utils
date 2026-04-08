use glam::{Vec3A, Vec4};

use super::wheel_info::WheelInfo;
use crate::bullet::{
    collision::StaticPlaneShape,
    dynamics::{
        contact_constraint::resolve_single_bilateral_simd,
        discrete_dynamics_world::DiscreteDynamicsWorld, rigid_body::RigidBody,
    }, linear_math::to_simd,
};

pub const NUM_WHEELS: usize = 4;

pub struct VehicleRL {
    pub wheels: [WheelInfo; NUM_WHEELS],
    pub engine_force: f32,
    pub brake: f32,
    pub lat_friction: [f32; 4],
    pub long_friction: [f32; 4],
}

impl VehicleRL {
    pub fn new(wheels: [WheelInfo; NUM_WHEELS]) -> Self {
        Self {
            wheels,
            engine_force: 0.0,
            brake: 0.0,
            lat_friction: [1.0; 4],
            long_friction: [1.0; 4],
        }
    }

    pub fn calc_friction_impulses_simd(&mut self, chassis: &RigidBody, friction_scale: f32) {
        const ROLLING_FRICTION_SCALE: f32 = 113.73963;

        let mut axle_x = [0.0; 4];
        let mut axle_y = [0.0; 4];
        let mut axle_z = [0.0; 4];
        let mut contact_x = [0.0; 4];
        let mut contact_y = [0.0; 4];
        let mut contact_z = [0.0; 4];

        for (i, wheel) in self.wheels.iter().enumerate() {
            let axle_dir = wheel.axle_dir;
            axle_x[i] = axle_dir.x;
            axle_y[i] = axle_dir.y;
            axle_z[i] = axle_dir.z;

            let contact = wheel.raycast_info.contact_point_ws;
            contact_x[i] = contact.x;
            contact_y[i] = contact.y;
            contact_z[i] = contact.z;
        }
        
        let [axle_x, axle_y, axle_z] = to_simd(&[axle_x, axle_y, axle_z]);
        let [contact_x, contact_y, contact_z] = to_simd(&[contact_x, contact_y, contact_z]);

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
        let impulse_x = total_x * scale;
        let impulse_y = total_y * scale;
        let impulse_z = total_z * scale;

        for i in 0..4 {
            self.wheels[i].impulse = Vec3A::new(impulse_x[i], impulse_y[i], impulse_z[i]);
        }
    }

    pub fn update_vehicle_first(&mut self, collision_world: &DiscreteDynamicsWorld) {
        let chassis = &collision_world.collision_obj;

        for wheel in &mut self.wheels[0..2] {
            wheel.update_wheel_trans::<true>(chassis.get_world_trans());
        }

        for wheel in &mut self.wheels[2..] {
            wheel.update_wheel_trans::<false>(chassis.get_world_trans());
        }

        let mut sources = [Vec3A::ZERO; 4];
        let mut targets = [Vec3A::ZERO; 4];

        for (i, wheel) in self.wheels.iter_mut().enumerate() {
            (sources[i], targets[i]) = wheel.get_raycast_info();
        }

        let ray_results = StaticPlaneShape::perform_raycast(&sources, &targets);

        for (i, wheel) in self.wheels.iter_mut().enumerate() {
            wheel.apply_ray_cast(chassis, ray_results[i]);
        }

        self.calc_friction_impulses_simd(chassis, chassis.mass / 3.0);
    }

    pub fn update_vehicle_second(&mut self, cb: &mut RigidBody, step: f32) {
        for wheel in &mut self.wheels {
            wheel.update_suspension(cb, step);
            wheel.apply_friction_impulses(cb, step);
        }
    }
}
