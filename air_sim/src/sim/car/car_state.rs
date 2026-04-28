use std::ops::{Deref, DerefMut};

use glam::{Mat3A, Quat, Vec3A};

use crate::{CarControls, PhysState};

#[derive(Clone, Copy, Debug)]
pub struct CarState {
    pub phys: PhysState,
    /// Controls to simulate the car with
    pub controls: CarControls,
    /// Controls from the last time this car was simulated (equals `controls` after step)
    pub prev_controls: CarControls,
    /// Whether we jumped to get into the air
    ///
    /// Can be false while airborne, if we left the ground with a flip reset
    pub has_jumped: bool,
    /// True if we have double jumped and are still in the air
    pub has_double_jumped: bool,
    /// True if we are in the air, and (have flipped or are currently flipping)
    pub has_flipped: bool,
    /// Relative torque direction of the flip
    ///
    /// Forward flip will have positive Y
    pub flip_rel_torque: Vec3A,
    /// When currently jumping, the time since we started jumping, else 0
    pub jump_time: f32,
    /// When currently flipping, the time since we started flipping, else 0
    pub flip_time: f32,
    /// True during a flip (not an auto-flip, and not after a flip)
    pub is_flipping: bool,
    /// True during a jump
    pub is_jumping: bool,
    /// Total time spent in the air
    pub air_time: f32,
    /// Time spent in the air once `!is_jumping`
    ///
    /// If we never jumped, it is 0
    pub air_time_since_jump: f32,
    /// Goes from 0 to 100
    pub boost: f32,
    /// Used for recharge boost, counts up from 0 on spawn (in seconds)
    pub time_since_boosted: f32,
    /// True if we boosted that tick
    ///
    /// There exists a minimum boosting time, thus why we must track boosting time
    pub is_boosting: bool,
    pub boosting_time: f32,
}

impl Default for CarState {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl CarState {
    pub const DEFAULT: Self = Self {
        phys: PhysState {
            pos: Vec3A::new(0.0, 0.0, crate::sim::consts::car::spawn::REST_Z),
            rot_mat: Mat3A::IDENTITY,
            rot_quat: Quat::IDENTITY,
            vel: Vec3A::ZERO,
            ang_vel: Vec3A::ZERO,
        },
        controls: CarControls::DEFAULT,
        prev_controls: CarControls::DEFAULT,
        has_jumped: false,
        has_double_jumped: false,
        has_flipped: false,
        flip_rel_torque: Vec3A::ZERO,
        jump_time: 0.0,
        flip_time: 0.0,
        is_flipping: false,
        is_jumping: false,
        air_time: 0.0,
        air_time_since_jump: 0.0,
        boost: crate::sim::consts::car::boost::SPAWN_AMOUNT,
        time_since_boosted: 0.0,
        is_boosting: false,
        boosting_time: 0.0,
    };

    #[must_use]
    pub const fn has_flip_or_jump(&self) -> bool {
        !self.has_flipped
            && !self.has_double_jumped
            && self.air_time_since_jump < crate::sim::consts::car::jump::DOUBLEJUMP_MAX_DELAY
    }
}

impl Deref for CarState {
    type Target = PhysState;
    fn deref(&self) -> &Self::Target {
        &self.phys
    }
}

impl DerefMut for CarState {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.phys
    }
}
