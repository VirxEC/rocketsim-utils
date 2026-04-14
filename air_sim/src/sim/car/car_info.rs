use crate::CarBodyConfig;

/// Immutable information attached to each car
#[derive(Clone, Copy, Debug, Default)]
pub struct CarInfo {
    pub idx: usize,
    pub config: CarBodyConfig,
}
