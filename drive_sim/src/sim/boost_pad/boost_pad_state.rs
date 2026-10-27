#[derive(Clone, Debug)]
pub struct BoostPadState {
    /// Car members in retained-entry order. A car index can occur on one pad only.
    pub touching_car_idxs: Vec<usize>,
    /// The last tick when we gave a car boost
    pub cooldown: f32,
}

impl Default for BoostPadState {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl BoostPadState {
    pub const DEFAULT: Self = Self {
        cooldown: 0.0,
        touching_car_idxs: Vec::new(),
    };

    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.cooldown <= 0.0
    }
}
