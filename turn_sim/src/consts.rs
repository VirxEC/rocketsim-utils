pub mod car {
    /// Car can never exceed this angular velocity (radians/s)
    pub const MAX_ANG_SPEED: f32 = 5.5;

    pub mod air_control {
        use std::f32::consts::TAU;

        use glam::Vec3A;

        pub const TORQUE: Vec3A = Vec3A::new(130., 95., 400.);
        pub const DAMPING: Vec3A = Vec3A::new(30., 20., 50.);
        pub const TORQUE_APPLY_SCALE: f32 = TAU / (1 << 16) as f32 * 1000.0;
    }
}
