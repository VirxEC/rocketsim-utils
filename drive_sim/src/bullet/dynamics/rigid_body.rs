use std::f32::consts::FRAC_PI_4;

use glam::{Affine3A, Mat3A, Quat, Vec3A, Vec4};

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Impulse {
    /// (lin_impulse)
    Linear(Vec3A),
    /// (lin_impulse, rel_pos_offset)
    LinearRelPos(Vec3A, Vec3A),
    /// (ang_impulse)
    Angular(Vec3A),
}

#[derive(Clone, Copy, Debug)]
pub struct RigidBody {
    world_trans: Affine3A,
    quat_trans: Quat,
    pub inv_inertia_tensor_world: Mat3A,
    pub lin_vel: Vec3A,
    pub ang_vel: Vec3A,
    pub mass: f32,
    pub inv_mass: f32,
    pub inv_inertia_local: Vec3A,

    pub accum_lin_vel: Vec3A,
    pub accum_ang_vel: Vec3A,
}

impl RigidBody {
    pub fn new(translation: Vec3A, local_inertia: Vec3A, mass: f32) -> Self {
        let world_trans = Affine3A {
            matrix3: Mat3A::IDENTITY,
            translation,
        };
        let inv_inertia_local = Vec3A::select(
            local_inertia.cmpeq(Vec3A::ZERO),
            Vec3A::ZERO,
            1.0 / local_inertia,
        );

        let inv_inertia_tensor_world =
            Self::get_inertia_tensor(world_trans.matrix3, inv_inertia_local);

        Self {
            mass,
            world_trans,
            quat_trans: Quat::IDENTITY,
            inv_mass: 1.0 / mass,
            inv_inertia_local,
            inv_inertia_tensor_world,
            lin_vel: Vec3A::ZERO,
            ang_vel: Vec3A::ZERO,
            accum_lin_vel: Vec3A::ZERO,
            accum_ang_vel: Vec3A::ZERO,
        }
    }

    #[inline]
    pub const fn get_world_trans(&self) -> &Affine3A {
        &self.world_trans
    }

    fn get_inertia_tensor(world_mat: Mat3A, inv_inertia_local: Vec3A) -> Mat3A {
        let mut scaled_mat = world_mat.transpose();
        scaled_mat.x_axis *= inv_inertia_local;
        scaled_mat.y_axis *= inv_inertia_local;
        scaled_mat.z_axis *= inv_inertia_local;

        world_mat * scaled_mat
    }

    pub fn update_inertia_tensor(&mut self) {
        self.inv_inertia_tensor_world =
            Self::get_inertia_tensor(self.world_trans.matrix3, self.inv_inertia_local);
    }

    /// Add an impulse of a given type
    ///
    /// `massed`: Scale down by `self.inv_mass`
    ///
    /// `accum`: Accumulate this impulse to be applied while
    /// stepping the simulation (instead of immediately)
    pub fn add_impulse(&mut self, impulse: Impulse, massed: bool, accum: bool) {
        let mut lin_impulse = Vec3A::ZERO;
        let mut ang_impulse = Vec3A::ZERO;

        let massed_scaler = if massed { self.inv_mass } else { 1.0 };
        match impulse {
            Impulse::Linear(v) => {
                lin_impulse = v * massed_scaler;
            }
            Impulse::LinearRelPos(v, rel_pos) => {
                lin_impulse = v * massed_scaler;
                ang_impulse = self.inv_inertia_tensor_world * rel_pos.cross(v);
                if !massed {
                    ang_impulse *= self.mass; // Have to undo the effects of inv inertia tensor
                }
            }
            Impulse::Angular(av) => {
                ang_impulse = av * massed_scaler;
            }
        };

        if accum {
            self.accum_lin_vel += lin_impulse;
            self.accum_ang_vel += ang_impulse;
        } else {
            self.lin_vel += lin_impulse;
            self.ang_vel += ang_impulse;
        }
    }

    #[inline]
    pub fn apply_impulses(
        &mut self,
        impulse_x: Vec4,
        impulse_y: Vec4,
        impulse_z: Vec4,
        rel_x: Vec4,
        rel_y: Vec4,
        rel_z: Vec4,
    ) {
        let total_impulse = Vec3A::new(
            impulse_x.element_sum(),
            impulse_y.element_sum(),
            impulse_z.element_sum(),
        );
        self.lin_vel += total_impulse * self.inv_mass;

        let torque_x = rel_y * impulse_z - rel_z * impulse_y;
        let torque_y = rel_z * impulse_x - rel_x * impulse_z;
        let torque_z = rel_x * impulse_y - rel_y * impulse_x;

        let torque = Vec3A::new(
            torque_x.element_sum(),
            torque_y.element_sum(),
            torque_z.element_sum(),
        );
        self.ang_vel += self.inv_inertia_tensor_world * torque;
    }

    pub fn integrate_trans(&mut self, tick_time: f32) {
        self.world_trans.translation += self.lin_vel * tick_time;

        let mut angle = self.ang_vel.length();

        let angular_motion_threshold = FRAC_PI_4 / tick_time;
        if angle > angular_motion_threshold {
            angle = angular_motion_threshold;
        }

        let axis = if angle < 0.001 {
            self.ang_vel
                * (0.5 * tick_time - tick_time * tick_time * tick_time * 0.020_833_334)
                * angle
                * angle
        } else {
            self.ang_vel * ((0.5 * angle * tick_time).sin() / angle)
        };

        let dorn = Quat::from_xyzw(axis.x, axis.y, axis.z, (angle * tick_time * 0.5).cos());
        self.quat_trans = (dorn * self.quat_trans).normalize();
        self.world_trans.matrix3 = Mat3A::from_quat(self.quat_trans);

        self.update_inertia_tensor();
    }

    pub fn set_center_of_mass_trans(&mut self, xform: Affine3A) {
        self.quat_trans = Quat::from_mat3a(&xform.matrix3);
        self.world_trans = xform;
        self.update_inertia_tensor();
    }

    pub fn get_vel_in_local_point(&self, rel_pos: Vec3A) -> Vec3A {
        self.lin_vel + self.ang_vel.cross(rel_pos)
    }

    #[inline]
    pub const fn clear_accum_vels(&mut self) {
        self.accum_lin_vel = Vec3A::ZERO;
        self.accum_ang_vel = Vec3A::ZERO;
    }

    #[inline]
    pub const fn get_up_vector(&self) -> Vec3A {
        self.get_world_trans().matrix3.z_axis
    }

    #[inline]
    pub fn get_forward_speed(&self) -> f32 {
        self.lin_vel.dot(self.get_world_trans().matrix3.x_axis)
    }

    #[inline]
    pub const fn get_world_pos(&self) -> Vec3A {
        self.get_world_trans().translation
    }

    pub fn limit_vels(&mut self, max_lin_speed: f32, max_ang_speed: f32) {
        if self.lin_vel.length_squared() > max_lin_speed.powi(2) {
            self.lin_vel = self.lin_vel.normalize_or_zero() * max_lin_speed;
        }

        if self.ang_vel.length_squared() > max_ang_speed.powi(2) {
            self.ang_vel = self.ang_vel.normalize_or_zero() * max_ang_speed;
        }
    }
}
