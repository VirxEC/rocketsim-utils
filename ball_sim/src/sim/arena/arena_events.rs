use glam::Vec3A;

#[derive(Debug, Copy, Clone)]
pub struct BallHitWorldEvent {
    pub contact_point: Vec3A,
    pub contact_normal: Vec3A,
}

#[derive(Debug, Clone)]
pub struct ArenaEventList {
    events: Vec<BallHitWorldEvent>,
}

impl Default for ArenaEventList {
    #[inline]
    fn default() -> Self {
        Self {
            events: Vec::with_capacity(Self::STARTING_CAPACITY),
        }
    }
}

impl ArenaEventList {
    const STARTING_CAPACITY: usize = 12;

    #[inline]
    pub fn push(&mut self, event: BallHitWorldEvent) {
        self.events.push(event);
    }

    #[inline]
    pub fn events(&self) -> &[BallHitWorldEvent] {
        &self.events
    }

    #[inline]
    pub fn clear(&mut self) {
        self.events.clear();
    }
}
