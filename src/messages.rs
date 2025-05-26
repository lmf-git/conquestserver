use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "camelCase")]
pub enum ClientMessage {
    Input {
        input: PlayerInput,
        sequence: u32,
    },
    Ping {
        timestamp: i64,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "camelCase")]
pub enum ServerMessage {
    Init {
        #[serde(rename = "playerId")]
        player_id: String,
        state: HashMap<String, PlayerState>,
    },
    State {
        state: HashMap<String, PlayerState>,
    },
    PlayerLeft {
        #[serde(rename = "playerId")]
        player_id: String,
    },
    Pong {
        timestamp: i64,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlayerInput {
    pub forward: bool,
    pub backward: bool,
    pub left: bool,
    pub right: bool,
    pub jump: bool,
    pub run: bool,
    pub yaw: f32,
    pub pitch: f32,
    pub world_position: [f32; 3],
    pub world_origin: [f32; 3],
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlayerState {
    pub position: [f32; 3],  // This is world position (local + origin)
    pub velocity: [f32; 3],
    pub rotation: [f32; 4], // Quaternion (x, y, z, w)
    #[serde(rename = "inputSequence")]
    pub input_sequence: u32,
    #[serde(rename = "isGrounded")]
    pub is_grounded: bool,
    #[serde(rename = "worldOrigin")]
    pub world_origin: [f32; 3],  // Add world origin so client knows the offset
}
