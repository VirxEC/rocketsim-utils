use glam::Vec3A;

#[derive(Debug, Copy, Clone)]
pub struct BallHitWorldEvent {
    pub contact_point: Vec3A,
    pub contact_normal: Vec3A,
}

#[derive(Debug, Clone)]
pub(crate) struct ArenaEventList {
    events: Vec<BallHitWorldEvent>,
}

impl ArenaEventList {
    const STARTING_CAPACITY: usize = 12;

    #[inline]
    pub fn new() -> ArenaEventList {
        ArenaEventList {
            events: Vec::with_capacity(Self::STARTING_CAPACITY),
        }
    }

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
