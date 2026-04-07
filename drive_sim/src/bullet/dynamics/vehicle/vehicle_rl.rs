use glam::Vec3A;

use super::wheel_info::WheelInfo;
use crate::bullet::{
    collision::StaticPlaneShape,
    dynamics::{discrete_dynamics_world::DiscreteDynamicsWorld, rigid_body::RigidBody},
};

pub const NUM_WHEELS: usize = 4;

pub struct VehicleRL {
    pub wheels: [WheelInfo; NUM_WHEELS],
}

impl VehicleRL {
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

        let friction_scale = chassis.mass / 3.0;
        WheelInfo::calc_friction_impulses_simd(&mut self.wheels, chassis, friction_scale);
    }

    pub fn update_vehicle_second(&mut self, cb: &mut RigidBody, step: f32) {
        for wheel in &mut self.wheels {
            wheel.update_suspension(cb, step);
        }

        // note: all suspension MUST be updated before impulses are applied
        for wheel in &mut self.wheels {
            wheel.apply_friction_impulses(cb, step);
        }
    }
}
