use glam::Vec3A;

use crate::{GameMode, sim::linear_piece_curve::LinearPieceCurve};

/// `BulletPhysics` Units (1m) to Unreal Units (2cm) conversion scale
pub(crate) const BT_TO_UU: f32 = 50.0;

/// Unreal Units (2cm) to `BulletPhysics` Units (1m) conversion scale
pub(crate) const UU_TO_BT: f32 = 1.0 / 50.0;

/// The z-velocity added by gravity each second
pub const GRAVITY_Z: f32 = -650.0;

pub mod car {
    pub const MASS_BT: f32 = 180.0;

    pub const MAX_SPEED: f32 = 2300.0;

    pub mod boost {
        pub const MAX: f32 = 100.0;
        pub const USED_PER_SECOND: f32 = MAX / 3.0;
        /// Minimum time we can be boosting for
        pub const MIN_TIME: f32 = 0.1;
        /// uu/s for vel (on the ground)
        pub const ACCEL_GROUND: f32 = 2975.0 / 3.0;
        pub const SPAWN_AMOUNT: f32 = MAX / 3.0;
        /// Amount of boost recharged per second when recharging
        pub const RECHARGE_PER_SECOND: f32 = 10.0;
        /// Delay after the car stops boosting
        pub const RECHARGE_DELAY: f32 = 0.25;
    }

    /// Car can never exceed this angular velocity (radians/s)
    pub const MAX_ANG_SPEED: f32 = 5.5;

    pub mod drive {
        pub const THROTTLE_TORQUE_AMOUNT: f32 = super::MASS_BT * 400.0;
        pub const BRAKE_TORQUE_AMOUNT: f32 = super::MASS_BT * (14.25 + (1.0 / 3.));
        /// If we are costing with less than this forward vel, we full-brake
        pub const STOPPING_FORWARD_VEL: f32 = 25.0;
        /// How much the brake is applied when coasting
        pub const COASTING_BRAKE_FACTOR: f32 = 0.15;
        /// If we are braking and moving faster than this, disable throttle
        pub const BRAKING_NO_THROTTLE_SPEED_THRESH: f32 = 0.01;
        /// Throttle input of less than this is ignored
        pub const THROTTLE_DEADZONE: f32 = 0.001;

        pub const POWERSLIDE_RISE_RATE: f32 = 5.0;
        pub const POWERSLIDE_FALL_RATE: f32 = 2.0;
    }

    pub mod spawn {
        pub const REST_Z: f32 = 17.0;
    }
}

pub mod bullet_vehicle {
    pub const SUSPENSION_FORCE_SCALE_FRONT: f32 = 36.0 - (1.0 / 4.);
    pub const SUSPENSION_FORCE_SCALE_BACK: f32 = 54.0 + (1.0 / 4.) + (1.5 / 100.);
    pub const SUSPENSION_STIFFNESS: f32 = 500.0;
    pub const WHEELS_DAMPING_COMPRESSION: f32 = 25.0;
    pub const WHEELS_DAMPING_RELAXATION: f32 = 40.0;
    pub const MAX_SUSPENSION_TRAVEL: f32 = 12.0;
    pub const SUSPENSION_SUBTRACTION: f32 = 0.05;
}

pub mod curves {
    use super::LinearPieceCurve;

    pub const STEER_ANGLE_FROM_SPEED: LinearPieceCurve<6> = LinearPieceCurve::new([
        (0., 0.53356),
        (500., 0.31930),
        (1000., 0.18203),
        (1500., 0.10570),
        (1750., 0.08507),
        (3000., 0.03454),
    ]);
    pub const POWERSLIDE_STEER_ANGLE_FROM_SPEED: LinearPieceCurve<2> =
        LinearPieceCurve::new([(0., 0.39235), (2500., 0.12610)]);
    pub const DRIVE_SPEED_TORQUE_FACTOR: LinearPieceCurve<3> =
        LinearPieceCurve::new([(0., 1.0), (1400., 0.1), (1410., 0.0)]);
    pub const LAT_FRICTION: LinearPieceCurve<2> = LinearPieceCurve::new([(0., 1.0), (1., 0.2)]);
    pub const HANDBRAKE_LAT_FRICTION_FACTOR: f32 = 0.9;
    pub const HANDBRAKE_LONG_FRICTION_FACTOR: LinearPieceCurve<2> =
        LinearPieceCurve::new([(0., 0.5), (1., 0.9)]);
}

pub mod boost_pads {
    use super::{GameMode, Vec3A};

    // TODO: Do something about repetitive small/big pairs
    pub const CYL_HEIGHT: f32 = 95.0;
    pub const CYL_RAD_BIG: f32 = 208.0;
    pub const CYL_RAD_SMALL: f32 = 144.0;
    pub const BOX_HEIGHT: f32 = 64.0;
    pub const BOX_RAD_BIG: f32 = 160.0;
    pub const BOX_RAD_SMALL: f32 = 120.0;
    pub const COOLDOWN_BIG: f32 = 10.0;
    pub const COOLDOWN_SMALL: f32 = 4.0;
    pub const BOOST_AMOUNT_BIG: f32 = 100.0;
    pub const BOOST_AMOUNT_SMALL: f32 = 12.0;

    pub const fn get_locations(game_mode: GameMode, is_big: bool) -> &'static [Vec3A] {
        const LOCS_SMALL_SOCCAR: [Vec3A; 28] = [
            Vec3A::new(0., -4240., 70.),
            Vec3A::new(-1792., -4184., 70.),
            Vec3A::new(1792., -4184., 70.),
            Vec3A::new(-940., -3308., 70.),
            Vec3A::new(940., -3308., 70.),
            Vec3A::new(0., -2816., 70.),
            Vec3A::new(-3584., -2484., 70.),
            Vec3A::new(3584., -2484., 70.),
            Vec3A::new(-1788., -2300., 70.),
            Vec3A::new(1788., -2300., 70.),
            Vec3A::new(-2048., -1036., 70.),
            Vec3A::new(0., -1024., 70.),
            Vec3A::new(2048., -1036., 70.),
            Vec3A::new(-1024., 0., 70.),
            Vec3A::new(1024., 0., 70.),
            Vec3A::new(-2048., 1036., 70.),
            Vec3A::new(0., 1024., 70.),
            Vec3A::new(2048., 1036., 70.),
            Vec3A::new(-1788., 2300., 70.),
            Vec3A::new(1788., 2300., 70.),
            Vec3A::new(-3584., 2484., 70.),
            Vec3A::new(3584., 2484., 70.),
            Vec3A::new(0., 2816., 70.),
            Vec3A::new(-940., 3308., 70.),
            Vec3A::new(940., 3308., 70.),
            Vec3A::new(-1792., 4184., 70.),
            Vec3A::new(1792., 4184., 70.),
            Vec3A::new(0., 4240., 70.),
        ];
        const LOCS_BIG_SOCCAR: [Vec3A; 6] = [
            Vec3A::new(-3584., 0., 73.),
            Vec3A::new(3584., 0., 73.),
            Vec3A::new(-3072., 4096., 73.),
            Vec3A::new(3072., 4096., 73.),
            Vec3A::new(-3072., -4096., 73.),
            Vec3A::new(3072., -4096., 73.),
        ];

        const LOCS_SMALL_HOOPS: [Vec3A; 14] = [
            Vec3A::new(1536., -1024., 64.),
            Vec3A::new(-1280., -2304., 64.),
            Vec3A::new(0., -2816., 64.),
            Vec3A::new(-1536., -1024., 64.),
            Vec3A::new(1280., -2304., 64.),
            Vec3A::new(-512., 512., 64.),
            Vec3A::new(-1536., 1024., 64.),
            Vec3A::new(1536., 1024., 64.),
            Vec3A::new(1280., 2304., 64.),
            Vec3A::new(0., 2816., 64.),
            Vec3A::new(512., 512., 64.),
            Vec3A::new(512., -512., 64.),
            Vec3A::new(-512., -512., 64.),
            Vec3A::new(-1280., 2304., 64.),
        ];
        const LOCS_BIG_HOOPS: [Vec3A; 6] = [
            Vec3A::new(-2176., 2944., 72.),
            Vec3A::new(2176., -2944., 72.),
            Vec3A::new(-2176., -2944., 72.),
            Vec3A::new(-2432., 0., 72.),
            Vec3A::new(2432., 0., 72.),
            Vec3A::new(2175.99, 2944., 72.),
        ];

        match game_mode {
            GameMode::Hoops => {
                if is_big {
                    &LOCS_BIG_HOOPS
                } else {
                    &LOCS_SMALL_HOOPS
                }
            }
            _ => {
                if is_big {
                    &LOCS_BIG_SOCCAR
                } else {
                    &LOCS_SMALL_SOCCAR
                }
            }
        }
    }
}
