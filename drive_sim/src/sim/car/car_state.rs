use std::ops::{Deref, DerefMut};

use glam::{Mat3A, Vec3A};

use crate::{CarControls, PhysState, sim::consts::car};

#[derive(Clone, Copy, Debug)]
pub struct CarState {
    pub phys: PhysState,
    /// Controls to simulate the car with
    pub controls: CarControls,
    /// Goes from 0 to 100
    pub boost: f32,
    /// Used for recharge boost, counts up from 0 on spawn (in seconds)
    pub time_since_boosted: f32,
    /// True if we boosted that tick
    ///
    /// There exists a minimum boosting time, thus why we must track boosting time
    pub is_boosting: bool,
    pub boosting_time: f32,
    /// This is a state variable due to the rise/fall rate of handbrake inputs
    pub handbrake_val: f32,
}

impl Default for CarState {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl CarState {
    pub const DEFAULT: Self = Self {
        phys: PhysState {
            pos: Vec3A::new(0.0, 0.0, car::spawn::REST_Z),
            rot_mat: Mat3A::IDENTITY,
            vel: Vec3A::ZERO,
            ang_vel: Vec3A::ZERO,
        },
        controls: CarControls::DEFAULT,
        boost: car::boost::SPAWN_AMOUNT,
        time_since_boosted: 0.0,
        is_boosting: false,
        boosting_time: 0.0,
        handbrake_val: 0.0,
    };
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
