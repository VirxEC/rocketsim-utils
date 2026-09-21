pub struct PhysicsCoefs {
    pub friction: f32,
    pub restitution: f32,
}

/// `BulletPhysics` Units (1m) to Unreal Units (2cm) conversion scale
pub(crate) const BT_TO_UU: f32 = 50.0;

/// Unreal Units (2cm) to `BulletPhysics` Units (1m) conversion scale
pub(crate) const UU_TO_BT: f32 = 1.0 / 50.0;

/// The z-velocity added by gravity each second
pub const GRAVITY_Z: f32 = -650.0;

pub mod car {
    use super::PhysicsCoefs;

    pub const MASS_BT: f32 = 180.0;

    pub const BASE_COEFS: PhysicsCoefs = PhysicsCoefs {
        friction: 0.3,
        restitution: 0.1,
    };

    pub const MAX_SPEED: f32 = 2300.0;

    pub mod boost {
        pub const MAX: f32 = 100.0;
        /// Boost is consumed at 0.333 * 100 per second - not MAX / 3.
        pub const USED_PER_SECOND: f32 = 33.3;
        /// Minimum time we can be boosting for
        pub const MIN_TIME: f32 = 0.1;
        /// uu/s for vel (airborne, paired with ground)
        pub const ACCEL_AIR: f32 = 2975.0 / 3.0;
        pub const SPAWN_AMOUNT: f32 = MAX / 3.0;
        /// Amount of boost recharged per second when recharging
        pub const RECHARGE_PER_SECOND: f32 = 10.0;
        /// Delay after the car stops boosting
        pub const RECHARGE_DELAY: f32 = 0.25;
    }

    /// Car can never exceed this angular velocity (radians/s)
    pub const MAX_ANG_SPEED: f32 = 5.5;

    pub mod supersonic {
        pub const START_SPEED: f32 = 2200.0;
        pub const MAINTAIN_MIN_SPEED: f32 = START_SPEED - 100.0;
        pub const MAINTAIN_MAX_TIME: f32 = 1.0;
    }

    pub mod drive {
        pub const THROTTLE_AIR_ACCEL: f32 = 200.0 / 3.0;
    }

    pub mod jump {
        pub const ACCEL: f32 = 4375.0 / 3.0;
        pub const IMMEDIATE_FORCE: f32 = 875.0 / 3.0;
        /// Minimum hold duration: 3 ticks (0.025 s @ 120 Hz)
        pub const MIN_TICKS: u32 = 3;
        /// Maximum hold duration: 24 ticks (0.2 s @ 120 Hz)
        pub const MAX_TICKS: u32 = 24;
        /// Can be at most 1.25 seconds after the jump is finished
        pub const DOUBLEJUMP_MAX_DELAY: f32 = 1.25;
    }

    pub mod flip {
        use glam::Vec3A;

        pub const Z_DAMP_120: f32 = 0.35;
        pub const Z_DAMP_START: f32 = 0.15;
        pub const Z_DAMP_END: f32 = 0.21;
        pub const TORQUE_TIME: f32 = 0.65;
        pub const TORQUE_MIN_TIME: f32 = 0.41;
        pub const PITCHLOCK_TIME: f32 = 1.0;
        pub const PITCHLOCK_EXTRA_TIME: f32 = 0.3;
        pub const INITIAL_VEL_SCALE: f32 = 500.0;
        /// X: Left/Right
        /// Y: Forward/Backward
        pub const TORQUE: Vec3A = Vec3A::new(260.0, 224.0, 0.0);
        pub const FORWARD_IMPULSE_MAX_SPEED_SCALE: f32 = 1.0;
        pub const SIDE_IMPULSE_MAX_SPEED_SCALE: f32 = 1.9;
        pub const BACKWARD_IMPULSE_MAX_SPEED_SCALE: f32 = 2.5;
        pub const BACKWARD_IMPULSE_SCALE_X: f32 = 16.0 / 15.0;
    }

    pub mod air_control {
        use std::f32::consts::PI;

        use glam::Vec3A;

        pub const TORQUE: Vec3A = Vec3A::new(130., 95., 400.);
        pub const DAMPING: Vec3A = Vec3A::new(30., 20., 50.);
        pub const TORQUE_APPLY_SCALE: f32 = 2.0 * PI / (1 << 16) as f32 * 1000.0;
    }

    pub mod spawn {
        pub const REST_Z: f32 = 17.0;
    }
}

pub mod quantize {
    pub const POS_SCALE: f32 = 100.0;
    pub const VEL_SCALE: f32 = 100.0;
    pub const ANG_VEL_SCALE: f32 = 10000.0;
}
