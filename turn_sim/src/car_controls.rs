#[derive(Debug, Clone, Copy)]
pub struct CarControls {
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
}

impl Default for CarControls {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl CarControls {
    pub const DEFAULT: Self = Self {
        pitch: 0.0,
        yaw: 0.0,
        roll: 0.0,
    };

    pub const fn clamp(mut self) -> Self {
        self.pitch = self.pitch.clamp(-1.0, 1.0);
        self.yaw = self.yaw.clamp(-1.0, 1.0);
        self.roll = self.roll.clamp(-1.0, 1.0);
        self
    }

    pub const fn with_pitch(mut self, pitch: f32) -> Self {
        self.pitch = pitch;
        self
    }

    pub const fn with_yaw(mut self, yaw: f32) -> Self {
        self.yaw = yaw;
        self
    }

    pub const fn with_roll(mut self, roll: f32) -> Self {
        self.roll = roll;
        self
    }
}
