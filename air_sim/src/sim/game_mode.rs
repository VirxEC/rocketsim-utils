#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum GameMode {
    #[default]
    Soccar,
    Hoops,
    Heatseeker,
    Snowday,
    Dropshot,
    /// Soccar but without goals, boost pads, or the arena hull. The cars and ball will fall infinitely.
    TheVoid,
}

impl GameMode {
    const NAMES: [&'static str; 6] = [
        "soccar",
        "hoops",
        "heatseeker",
        "snowday",
        "dropshot",
        "void",
    ];

    pub const fn name(self) -> &'static str {
        Self::NAMES[self as usize]
    }
}
