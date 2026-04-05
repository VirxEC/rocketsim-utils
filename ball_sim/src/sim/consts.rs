pub struct PhysicsCoefs {
    pub friction: f32,
    pub restitution: f32,
}

/// `BulletPhysics` Units (1m) to Unreal Units (2cm) conversion scale
pub(crate) const BT_TO_UU: f32 = 50.0;

/// Unreal Units (2cm) to `BulletPhysics` Units (1m) conversion scale
pub(crate) const UU_TO_BT: f32 = 1.0 / 50.0;

/// How many ticks occur in one second
pub const TICK_RATE: f32 = 120.0;

/// The amount of time each tick takes up (inverse of TICK_RATE)
pub const TICK_TIME: f32 = 1.0 / TICK_RATE;

/// The z-velocity added by gravity each second
pub const GRAVITY_Z: f32 = -650.0;

pub mod arena {
    use glam::Vec3A;

    use super::PhysicsCoefs;
    use crate::{GameMode, shared::Aabb};

    pub const BASE_COEFS: PhysicsCoefs = PhysicsCoefs {
        friction: 0.6,
        restitution: 0.3,
    };

    pub const fn get_aabb(game_mode: GameMode) -> Aabb {
        let (max_x, max_y, max_z) = match game_mode {
            GameMode::Hoops => (8900.0 / 3.0, 3581.0, 1820.0),
            GameMode::Dropshot => (5075.0, 4592.0, 2024.0),

            // Soccar arena
            _ => (4096.0, 5120.0, 2048.0),
        };

        let floor_height = match game_mode {
            GameMode::Dropshot => 1.5,
            _ => 0.0,
        };

        Aabb::new(
            Vec3A::new(-max_x, -max_y, floor_height),
            Vec3A::new(max_x, max_y, max_z),
        )
    }
}

pub mod ball {
    use super::PhysicsCoefs;
    use crate::GameMode;

    pub const fn get_radius(game_mode: GameMode) -> f32 {
        pub const RADIUS_SOCCAR: f32 = 91.25;
        pub const RADIUS_HOOPS: f32 = 96.3831;
        pub const RADIUS_DROPSHOT: f32 = 100.2565;
        match game_mode {
            GameMode::Hoops => RADIUS_HOOPS,
            GameMode::Dropshot => RADIUS_DROPSHOT,
            GameMode::Snowday => f32::NAN,
            _ => RADIUS_SOCCAR,
        }
    }

    /// Ref: <https://www.reddit.com/r/RocketLeague/comments/bmje9l/comment/emxkwrl/?context=3>
    pub const MASS_BT: f32 = 30.0; // Ball mass divided by 6
    /// Greater than ball radius because of arena mesh collision margin
    pub const REST_Z: f32 = 93.15;
    /// Ball can never exceed this angular velocity (radians/s)
    pub const MAX_ANG_SPEED: f32 = 6.0;
    /// Net-velocity drag multiplier
    pub const DRAG: f32 = 0.03;
    pub const COEFS: PhysicsCoefs = PhysicsCoefs {
        friction: 0.35,
        restitution: 0.6,
    };

    /// Z impulse applied to hoops ball on kickoff
    pub const HOOPS_LAUNCH_Z_VEL: f32 = 1000.0;
    pub const HOOPS_LAUNCH_DELAY: f32 = 0.265;

    pub const MAX_SPEED: f32 = 6000.0;
}

pub mod goal {
    use glam::Vec3A;

    use crate::{Team, shared::Aabb};

    pub const SOCCAR_GOAL_SCORE_BASE_THRESHOLD_Y: f32 = 5124.25;
    pub const SOCCAR_GOAL_HEIGHT: f32 = 642.775; // https://wiki.rlbot.org/v4/botmaking/useful-game-values/
    pub const SOCCAR_GOAL_HALF_WIDTH: f32 = 892.755; // https://wiki.rlbot.org/v4/botmaking/useful-game-values/
    pub const SOCCAR_GOAL_DEPTH: f32 = 880.0; // https://wiki.rlbot.org/v4/botmaking/useful-game-values/

    pub const HOOPS_GOAL_SCORE_THRESHOLD_Z: f32 = 270.0;

    pub const fn get_goal_aabb(goal_team: Team) -> Aabb {
        const FRONT_Y: f32 = SOCCAR_GOAL_SCORE_BASE_THRESHOLD_Y;
        const DEPTH: f32 = SOCCAR_GOAL_DEPTH;
        let (min_y, max_y) = if goal_team.is_blue() {
            (-FRONT_Y - DEPTH, -FRONT_Y)
        } else {
            (FRONT_Y, FRONT_Y + DEPTH)
        };
        Aabb::new(
            Vec3A::new(-SOCCAR_GOAL_HALF_WIDTH, min_y, 0.0),
            Vec3A::new(SOCCAR_GOAL_HALF_WIDTH, max_y, SOCCAR_GOAL_HEIGHT),
        )
    }

    pub const fn get_goal_face_center(goal_team: Team) -> Vec3A {
        Vec3A::new(
            0.0,
            SOCCAR_GOAL_SCORE_BASE_THRESHOLD_Y * goal_team.get_y_dir(),
            SOCCAR_GOAL_HEIGHT / 2.0,
        )
    }
}

pub mod heatseeker {
    use std::f32::consts::PI;

    use glam::Vec3A;

    /// Initial target speed from kickoff (goes to 2985 after the first touch)
    pub const INITIAL_TARGET_SPEED: f32 = 2900.0;
    /// Increase of target speed each touch
    pub const TARGET_SPEED_INCREMENT: f32 = 85.0;
    /// Minimum time between touches to speed up
    pub const MIN_SPEEDUP_INTERVAL: f32 = 1.0;
    /// Y of target point in goal
    pub const TARGET_Y: f32 = 5120.0;
    /// Height of target point in goal
    pub const TARGET_Z: f32 = 320.0;
    /// Interpolation of horizontal (X+Y) turning
    pub const HORIZONTAL_BLEND: f32 = 1.45;
    /// Interpolation of vertical (Z) turning
    pub const VERTICAL_BLEND: f32 = 0.78;
    /// Interpolation of acceleration towards target speed
    pub const SPEED_BLEND: f32 = 0.3;
    /// Maximum pitch angle of turning
    pub const MAX_TURN_PITCH: f32 = 7000.0 * PI / (1 << 15) as f32;
    /// Maximum speed the ball can seek at (different from `BALL_MAX_SPEED`)
    pub const MAX_SPEED: f32 = 4600.0;
    /// Threshold of wall collision Y backwall distance to change goal targets
    pub const WALL_BOUNCE_CHANGE_Y_THRESH: f32 = 300.0;
    /// Threshold of Y normal to trigger bounce-back
    pub const WALL_BOUNCE_CHANGE_Y_NORMAL: f32 = 0.5;
    /// Scale of the extra wall bounce impulse
    pub const WALL_BOUNCE_FORCE_SCALE: f32 = 1.0 / 3.0;
    /// Fraction of upward bounce impulse that goes straight up
    pub const WALL_BOUNCE_UP_FRAC: f32 = 0.3;
    pub const BALL_START_POS: Vec3A = Vec3A::new(-1000., -2220., 92.75);
    pub const BALL_START_VEL: Vec3A = Vec3A::new(0., -65., 650.);
}

pub mod snowday {
    use super::*;

    /// Real puck radius varies a bit from point to point, but it shouldn't matter
    pub const PUCK_RADIUS: f32 = 114.25;
    pub const PUCK_HEIGHT: f32 = 62.5;
    /// Number of points on each circle of the cylinder
    pub const PUCK_CIRCLE_POINT_AMOUNT: f32 = 20.0;
    pub const PUCK_MASS_BT: f32 = 50.0;
    pub const PUCK_GROUND_STICK_FORCE: f32 = 70.0;
    pub const PUCK_COEFS: PhysicsCoefs = PhysicsCoefs {
        friction: 0.1,
        restitution: 0.3,
    };
}

pub mod dropshot {
    pub const BALL_LAUNCH_Z_VEL: f32 = 985.0;
    pub const BALL_LAUNCH_DELAY: f32 = 0.26;
}
