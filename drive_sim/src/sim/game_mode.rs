#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum GameMode {
    #[default]
    Soccar,
    Hoops,
    Heatseeker,
    Snowday,
    Dropshot,
}

impl GameMode {
    const NAMES: [&'static str; 5] = ["soccar", "hoops", "heatseeker", "snowday", "dropshot"];

    pub const fn name(self) -> &'static str {
        Self::NAMES[self as usize]
    }
}
