#[derive(Debug, Clone, Copy)]
pub struct CarControls {
    pub throttle: f32,
    pub steer: f32,
    pub boost: bool,
    pub handbrake: bool,
}

impl Default for CarControls {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl CarControls {
    pub const DEFAULT: Self = Self {
        throttle: 0.0,
        steer: 0.0,
        boost: false,
        handbrake: false,
    };

    #[must_use]
    pub const fn clamp(mut self) -> Self {
        self.throttle = self.throttle.clamp(-1.0, 1.0);
        self.steer = self.steer.clamp(-1.0, 1.0);
        self
    }

    #[must_use]
    pub const fn with_throttle(mut self, val: f32) -> Self {
        self.throttle = val;
        self
    }

    #[must_use]
    pub const fn with_steer(mut self, val: f32) -> Self {
        self.steer = val;
        self
    }

    #[must_use]
    pub const fn with_boost(mut self, val: bool) -> Self {
        self.boost = val;
        self
    }

    #[must_use]
    pub const fn with_handbrake(mut self, val: bool) -> Self {
        self.handbrake = val;
        self
    }
}
