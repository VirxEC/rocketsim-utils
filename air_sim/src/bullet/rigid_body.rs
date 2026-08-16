use glam::{Affine3A, Mat3A, Quat, Vec3A};

use crate::bullet::transform_util::integrate_trans;

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Impulse {
    /// (lin_impulse)
    Linear(Vec3A),
    /// (lin_impulse, rel_pos_offset)
    LinearRelPos(Vec3A, Vec3A),
    /// (ang_impulse)
    Angular(Vec3A),
}

#[derive(Clone, Copy)]
pub struct RigidBody {
    pub world_trans: Affine3A,
    pub world_rotation: Quat,
    pub inv_inertia_tensor_world: Mat3A,
    pub lin_vel: Vec3A,
    pub ang_vel: Vec3A,
    pub mass: f32,
    pub inv_mass: f32,
    pub gravity: Vec3A,
    pub inv_inertia_local: Vec3A,
    pub accum_lin_vel: Vec3A,
    pub accum_ang_vel: Vec3A,
}

impl RigidBody {
    pub fn new(mass: f32, gravity: Vec3A, local_inertia: Vec3A) -> Self {
        let inv_mass = 1.0 / mass;

        let inv_inertia_local = Vec3A::select(
            local_inertia.cmpeq(Vec3A::ZERO),
            Vec3A::ZERO,
            1.0 / local_inertia,
        );

        let inv_inertia_tensor_world = Self::get_inertia_tensor(Mat3A::IDENTITY, inv_inertia_local);

        Self {
            world_trans: Affine3A::IDENTITY,
            world_rotation: Quat::IDENTITY,
            inv_inertia_tensor_world,
            lin_vel: Vec3A::ZERO,
            ang_vel: Vec3A::ZERO,
            mass,
            inv_mass,
            gravity,
            inv_inertia_local,
            accum_lin_vel: Vec3A::ZERO,
            accum_ang_vel: Vec3A::ZERO,
        }
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

    pub fn integration_trans(&mut self, time_step: f32) {
        self.world_trans.translation += self.lin_vel * time_step;
        integrate_trans(&mut self.world_rotation, self.ang_vel, time_step);
        self.world_trans.matrix3 = Mat3A::from_quat(self.world_rotation);
        self.update_inertia_tensor();
    }

    pub fn set_center_of_mass_trans(&mut self, xform: Affine3A) {
        self.world_rotation = Quat::from_mat3a(&xform.matrix3);
        self.world_trans = xform;
        self.update_inertia_tensor();
    }

    pub const fn clear_accum_vels(&mut self) {
        self.accum_lin_vel = Vec3A::ZERO;
        self.accum_ang_vel = Vec3A::ZERO;
    }

    pub fn get_forward_speed(&self) -> f32 {
        self.lin_vel.dot(self.world_trans.matrix3.x_axis)
    }

    pub fn step_simulation(&mut self, time_step: f32) {
        self.add_impulse(Impulse::Linear(self.gravity * time_step), false, true);

        self.lin_vel += self.accum_lin_vel;
        self.ang_vel += self.accum_ang_vel;

        self.integration_trans(time_step);
        self.clear_accum_vels();
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
