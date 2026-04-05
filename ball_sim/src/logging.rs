use std::io::Write;

use env_logger::WriteStyle;
use log::LevelFilter;

pub fn try_init() -> Result<(), log::SetLoggerError> {
    env_logger::builder()
        .format(|buf, record| writeln!(buf, "[RSIM | {}] {}", record.level(), record.args()))
        .write_style(WriteStyle::Always)
        .filter(None, LevelFilter::Info)
        .try_init()
}
