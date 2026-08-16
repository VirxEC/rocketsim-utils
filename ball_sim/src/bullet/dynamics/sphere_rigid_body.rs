use glam::Vec3A;

use crate::bullet::collision::shapes::sphere_shape::SphereShape;

pub struct SphereRigidBodyConstructionInfo {
    pub mass: f32,
    pub start_world_trans: Vec3A,
    pub collision_shape: SphereShape,
    pub local_inertia: Vec3A,
    pub linear_damping: f32,
    pub friction: f32,
    pub restitution: f32,
}

impl SphereRigidBodyConstructionInfo {
    pub const fn new(mass: f32, collision_shape: SphereShape) -> Self {
        Self {
            mass,
            collision_shape,
            local_inertia: Vec3A::ZERO,
            linear_damping: 0.0,
            friction: 0.5,
            restitution: 0.0,
            start_world_trans: Vec3A::ZERO,
        }
    }
}

/// An impulse to apply to a rigid body
#[derive(Debug, Copy, Clone, PartialEq)]
#[allow(dead_code)] // The other variants exist for API parity with RocketSim's `Impulse`
pub enum Impulse {
    /// (lin_impulse)
    Linear(Vec3A),
    /// (lin_impulse, rel_pos_offset)
    LinearRelPos(Vec3A, Vec3A),
    /// (ang_impulse)
    Angular(Vec3A),
}

#[derive(Clone, Copy, Debug)]
pub struct SphereRigidBody {
    world_trans: Vec3A,
    shape: SphereShape,
    pub interp_world_trans: Vec3A,
    pub friction: f32,
    pub restitution: f32,
    pub lin_vel: Vec3A,
    pub ang_vel: Vec3A,
    pub mass: f32,
    pub inv_mass: f32,
    pub inv_inertia_local: Vec3A,
    pub accum_lin_vel: Vec3A,
    pub accum_ang_vel: Vec3A,
    pub linear_damping: f32,
    pub inv_mass_splat: Vec3A,
}

impl SphereRigidBody {
    pub fn new(info: SphereRigidBodyConstructionInfo) -> Self {
        let inv_mass = if info.mass == 0.0 {
            0.0
        } else {
            1.0 / info.mass
        };

        let linear_damping = info.linear_damping.clamp(0.0, 1.0);

        let inv_inertia_local = Vec3A::select(
            info.local_inertia.cmpeq(Vec3A::ZERO),
            Vec3A::ZERO,
            1.0 / info.local_inertia,
        );

        Self {
            world_trans: info.start_world_trans,
            interp_world_trans: info.start_world_trans,
            shape: info.collision_shape,
            friction: info.friction,
            restitution: info.restitution,
            lin_vel: Vec3A::ZERO,
            ang_vel: Vec3A::ZERO,
            mass: info.mass,
            inv_mass,
            inv_inertia_local,
            accum_lin_vel: Vec3A::ZERO,
            accum_ang_vel: Vec3A::ZERO,
            linear_damping,
            inv_mass_splat: Vec3A::splat(inv_mass),
        }
    }

    pub const fn set_world_trans(&mut self, world_trans: Vec3A) {
        self.world_trans = world_trans;
    }

    pub const fn get_world_trans(&self) -> Vec3A {
        self.world_trans
    }

    pub const fn get_collision_shape(&self) -> &SphereShape {
        &self.shape
    }

    pub fn set_lin_vel(&mut self, lin_vel: Vec3A) {
        debug_assert!(!lin_vel.is_nan());
        self.lin_vel = lin_vel;
    }

    pub fn set_ang_vel(&mut self, ang_vel: Vec3A) {
        debug_assert!(!ang_vel.is_nan());
        self.ang_vel = ang_vel;
    }

    /// Add an impulse of a given type
    ///
    /// `massed`: Scale down by `self.inv_mass`
    ///
    /// `accum`: Accumulate this impulse to be applied while
    /// stepping the simulation (instead of immediately)
    #[inline(always)] // Should assure const evaluation
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
                ang_impulse = self.inv_inertia_local * rel_pos.cross(v);
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

    pub fn apply_damping(&mut self, time_step: f32) {
        if self.linear_damping != 0.0 {
            self.lin_vel *= (1.0 - self.linear_damping).powf(time_step);
        }
    }

    pub fn predict_integration_trans(&self, time_step: f32) -> Vec3A {
        self.world_trans + self.lin_vel * time_step
    }

    pub const fn set_center_of_mass_trans(&mut self, xform: Vec3A) {
        self.interp_world_trans = xform;
        self.set_world_trans(xform);
    }

    pub fn get_vel_in_local_point(&self, rel_pos: Vec3A) -> Vec3A {
        self.lin_vel + self.ang_vel.cross(rel_pos)
    }

    pub const fn clear_accum_vels(&mut self) {
        self.accum_lin_vel = Vec3A::ZERO;
        self.accum_ang_vel = Vec3A::ZERO;
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
