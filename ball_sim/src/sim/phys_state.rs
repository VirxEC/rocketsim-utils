use std::fmt::Display;

use glam::Vec3A;

#[derive(Clone, Copy, Debug)]
pub struct PhysState {
    pub pos: Vec3A,
    pub vel: Vec3A,
    pub ang_vel: Vec3A,
}

impl PhysState {
    #[must_use]
    pub fn flip_y(mut self) -> Self {
        const INVERT_SCALE: Vec3A = Vec3A::new(-1.0, -1.0, 1.0);

        self.pos *= INVERT_SCALE;
        self.vel *= INVERT_SCALE;
        self.ang_vel *= INVERT_SCALE;

        self
    }

    #[must_use]
    pub fn mirror_x(mut self) -> Self {
        const FLIP_SCALES: Vec3A = Vec3A::new(-1.0, 1.0, 1.0);

        self.pos *= FLIP_SCALES;
        self.vel *= FLIP_SCALES;

        self.ang_vel *= -FLIP_SCALES;

        self
    }

    #[must_use]
    pub fn mirror_y(mut self) -> Self {
        const FLIP_SCALES: Vec3A = Vec3A::new(1.0, -1.0, 1.0);

        self.pos *= FLIP_SCALES;
        self.vel *= FLIP_SCALES;

        self.ang_vel *= -FLIP_SCALES;

        self
    }
}

impl Display for PhysState {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.write_str("PhysState {")?;
        f.write_fmt(format_args!("\n\tpos: {}", self.pos))?;
        f.write_fmt(format_args!("\n\tvel: {}", self.vel))?;
        f.write_fmt(format_args!("\n\tang_vel: {}", self.ang_vel))?;
        f.write_str("}")
    }
}
