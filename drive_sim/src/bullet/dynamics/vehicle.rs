use glam::{Quat, Vec3A, Vec4};

use crate::{
    bullet::{
        collision::StaticPlaneShape,
        dynamics::{contact_constraint::resolve_single_bilateral, rigid_body::RigidBody},
    },
    consts::{
        UU_TO_BT,
        bullet_vehicle::{self, SUSPENSION_FORCE_SCALE_BACK, SUSPENSION_FORCE_SCALE_FRONT},
    },
};

pub const NUM_WHEELS: usize = 4;
const SUSPENSION_TRAVEL: f32 = bullet_vehicle::MAX_SUSPENSION_TRAVEL * UU_TO_BT;

fn vec_to_simd(vecs: [Vec3A; 4]) -> [Vec4; 3] {
    [
        Vec4::new(vecs[0].x, vecs[1].x, vecs[2].x, vecs[3].x),
        Vec4::new(vecs[0].y, vecs[1].y, vecs[2].y, vecs[3].y),
        Vec4::new(vecs[0].z, vecs[1].z, vecs[2].z, vecs[3].z),
    ]
}

#[derive(Default)]
pub struct RaycastInfo {
    pub hard_point_ws: [Vec4; 3],
    contact_point_ws: [Vec4; 3],
    wheel_direction_ws: Vec3A,
}

#[derive(Default)]
pub struct VehicleRL {
    pub steering_orn: [Quat; 2],
    pub raycast_info: RaycastInfo,
    pub chassis_connection_point_cs: [Vec4; 3],
    pub engine_force: f32,
    pub brake: f32,
    pub suspension_rest_length_1: Vec4,
    pub axle_dir: [Vec4; 3],
    suspension_length: Vec4,
    suspension_relative_vel: Vec4,
    pub wheel_radius: Vec4,
    pub lat_friction: Vec4,
    pub long_friction: Vec4,
    impulse: [Vec4; 3],
}

impl VehicleRL {
    fn calc_friction_impulses(&mut self, chassis: &RigidBody, friction_scale: f32) {
        const ROLLING_FRICTION_SCALE: f32 = 113.73963;

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

        let [axle_x, axle_y, axle_z] = self.axle_dir;
        let rel_vel = contact_vel_y * axle_x - contact_vel_x * axle_y + contact_vel_z * axle_z;

        let side_impulse =
            resolve_single_bilateral(chassis, rel_x, rel_y, rel_z, axle_x, axle_y, axle_z);

        let rolling_friction = if self.engine_force == 0.0 {
            let brake = Vec4::splat(self.brake);
            (rel_vel * ROLLING_FRICTION_SCALE).clamp(-brake, brake)
        } else {
            Vec4::splat(self.engine_force / friction_scale)
        };

        let rf = -rolling_friction * self.long_friction;
        let si = side_impulse * self.lat_friction;

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
        let ray_length = self.suspension_rest_length_1
            + self.wheel_radius
            + (SUSPENSION_TRAVEL - bullet_vehicle::SUSPENSION_SUBTRACTION);

        let target_x = self.raycast_info.hard_point_ws[0]
            + self.raycast_info.wheel_direction_ws.x * ray_length;
        let target_y = self.raycast_info.hard_point_ws[1]
            + self.raycast_info.wheel_direction_ws.y * ray_length;
        let target_z = self.raycast_info.hard_point_ws[2]
            + self.raycast_info.wheel_direction_ws.z * ray_length;

        [target_x, target_y, target_z]
    }

    fn apply_ray_casts(&mut self, chassis: &RigidBody) {
        let hard_z = self.raycast_info.hard_point_ws[2];
        let contact_z = self.raycast_info.contact_point_ws[2];

        let suspension_length = (hard_z - contact_z) - self.wheel_radius;

        let min_suspension_len = self.suspension_rest_length_1 - Vec4::splat(SUSPENSION_TRAVEL);
        let max_suspension_len = self.suspension_rest_length_1 + Vec4::splat(SUSPENSION_TRAVEL);
        self.suspension_length = suspension_length.clamp(min_suspension_len, max_suspension_len);

        let chassis_pos = chassis.get_world_trans().translation;
        let rel_x = self.raycast_info.contact_point_ws[0] - chassis_pos.x;
        let rel_y = self.raycast_info.contact_point_ws[1] - chassis_pos.y;

        let cross_z = chassis.ang_vel.x * rel_y - chassis.ang_vel.y * rel_x;
        self.suspension_relative_vel = cross_z + chassis.lin_vel.z;
    }

    fn update_wheel_trans(&mut self, chassis: &RigidBody) {
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

        let axle_dirs = [
            self.steering_orn[0] * chassis_trans.matrix3.y_axis,
            self.steering_orn[1] * chassis_trans.matrix3.y_axis,
            chassis_trans.matrix3.y_axis,
            chassis_trans.matrix3.y_axis,
        ];
        self.axle_dir = vec_to_simd(axle_dirs);
    }

    pub fn update_vehicle_first(&mut self, chassis: &RigidBody) {
        self.update_wheel_trans(chassis);

        let targets = self.get_raycast_targets();
        let ray_results =
            StaticPlaneShape::perform_raycast(&self.raycast_info.hard_point_ws, &targets);
        self.raycast_info.contact_point_ws = ray_results;

        self.apply_ray_casts(chassis);

        self.calc_friction_impulses(chassis, chassis.mass / 3.0);
    }

    fn update_suspension(&self, cb: &mut RigidBody, delta_time: f32) {
        const SUSPENSION_FORCE_SCALE: Vec4 = Vec4::new(
            SUSPENSION_FORCE_SCALE_FRONT,
            SUSPENSION_FORCE_SCALE_FRONT,
            SUSPENSION_FORCE_SCALE_BACK,
            SUSPENSION_FORCE_SCALE_BACK,
        );
        const COMPRESSION_DAMPING: Vec4 = Vec4::splat(bullet_vehicle::WHEELS_DAMPING_COMPRESSION);
        const RELAXATION_DAMPING: Vec4 = Vec4::splat(bullet_vehicle::WHEELS_DAMPING_RELAXATION);

        let force = (self.suspension_rest_length_1 - self.suspension_length)
            * bullet_vehicle::SUSPENSION_STIFFNESS;

        let damping_vel_scale = Vec4::select(
            self.suspension_relative_vel.cmplt(Vec4::ZERO),
            COMPRESSION_DAMPING,
            RELAXATION_DAMPING,
        );

        let suspension_force =
            (force - damping_vel_scale * self.suspension_relative_vel) * SUSPENSION_FORCE_SCALE;
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

    pub fn update_vehicle_second(&self, cb: &mut RigidBody, step: f32) {
        self.update_suspension(cb, step);
        self.apply_friction_impulses(cb, step);
    }
}
