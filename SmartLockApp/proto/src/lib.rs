use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
pub struct JsonLockCommand {
    pub lock: bool,
}

#[derive(Serialize, Deserialize)]
pub struct JsonBellState {
    pub bell: bool,
}
