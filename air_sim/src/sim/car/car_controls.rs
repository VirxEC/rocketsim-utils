use glam::Vec3;

#[derive(Debug, Clone, Copy)]
pub struct CarControls {
    pub throttle: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
    pub jump: bool,
    pub boost: bool,
}

impl Ord for CarControls {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // TODO: Surely there is a better way to do this
        macro_rules! cmp_field {
            ($field:ident) => {{
                // Explicitly copying because otherwise Rust yells at me
                let a = self.$field;
                let b = other.$field;
                a.total_cmp(&b)
            }};
        }

        cmp_field!(throttle)
            .then_with(|| cmp_field!(pitch))
            .then_with(|| cmp_field!(yaw))
            .then_with(|| cmp_field!(roll))
            .then_with(|| self.jump.cmp(&other.jump))
            .then_with(|| self.boost.cmp(&other.boost))
    }
}

impl PartialOrd for CarControls {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for CarControls {
    fn eq(&self, other: &Self) -> bool {
        self.throttle == other.throttle
            && self.pitch == other.pitch
            && self.yaw == other.yaw
            && self.roll == other.roll
            && self.jump == other.jump
            && self.boost == other.boost
    }
}

impl Eq for CarControls {}

impl Default for CarControls {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl CarControls {
    pub const DEFAULT: Self = Self {
        throttle: 0.0,
        pitch: 0.0,
        yaw: 0.0,
        roll: 0.0,
        jump: false,
        boost: false,
    };

    pub const NUM_VALS: usize = 8;

    pub const fn clamp(mut self) -> Self {
        self.throttle = self.throttle.clamp(-1.0, 1.0);
        self.pitch = self.pitch.clamp(-1.0, 1.0);
        self.yaw = self.yaw.clamp(-1.0, 1.0);
        self.roll = self.roll.clamp(-1.0, 1.0);
        self
    }

    pub const fn pyr(self) -> Vec3 {
        Vec3::new(self.pitch, self.yaw, self.roll)
    }

    pub const fn with_throttle(mut self, val: f32) -> Self {
        self.throttle = val;
        self
    }

    pub const fn with_pitch(mut self, val: f32) -> Self {
        self.pitch = val;
        self
    }

    pub const fn with_yaw(mut self, val: f32) -> Self {
        self.yaw = val;
        self
    }

    pub const fn with_roll(mut self, val: f32) -> Self {
        self.roll = val;
        self
    }

    pub const fn with_pyr(mut self, pyr: Vec3) -> Self {
        (self.pitch, self.yaw, self.roll) = (pyr.x, pyr.y, pyr.z);
        self
    }

    pub const fn with_jump(mut self, val: bool) -> Self {
        self.jump = val;
        self
    }

    pub const fn with_boost(mut self, val: bool) -> Self {
        self.boost = val;
        self
    }
}
