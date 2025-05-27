use crate::messages::{PlayerInfo, Position, Rotation, ServerMessage, Velocity};
use crate::physics::PhysicsWorld;
use axum::extract::ws::Message;
use dashmap::DashMap;
use nalgebra::Vector3;
use rapier3d::prelude::*;
use std::sync::Arc;
use tokio::sync::mpsc;
use uuid::Uuid;

pub struct Player {
    pub id: Uuid,
    pub position: Vector3<f32>,
    pub rotation: nalgebra::UnitQuaternion<f32>,
    pub velocity: Vector3<f32>,
    pub rigid_body_handle: Option<RigidBodyHandle>,
    pub collider_handle: Option<ColliderHandle>,
    pub sender: mpsc::UnboundedSender<Message>,
}

impl Player {
    pub fn new(id: Uuid, position: Vector3<f32>, sender: mpsc::UnboundedSender<Message>) -> Self {
        Self {
            id,
            position,
            rotation: nalgebra::UnitQuaternion::identity(),
            velocity: Vector3::zeros(),
            rigid_body_handle: None,
            collider_handle: None,
            sender,
        }
    }

    pub fn update_state(&mut self, pos: Position, rot: Rotation, vel: Velocity) {
        self.position = Vector3::new(pos.x, pos.y, pos.z);
        self.rotation = nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
            rot.w, rot.x, rot.y, rot.z,
        ));
        self.velocity = Vector3::new(vel.x, vel.y, vel.z);
    }

    pub async fn send_message(&self, msg: &ServerMessage) {
        if let Ok(json) = serde_json::to_string(msg) {
            let _ = self.sender.send(Message::Text(json));
        }
    }
}

pub struct PlayerManager {
    players: Arc<DashMap<Uuid, Player>>,
}

impl PlayerManager {
    pub fn new() -> Self {
        Self {
            players: Arc::new(DashMap::new()),
        }
    }

    pub fn add_player(&mut self, id: Uuid, position: Vector3<f32>) {
        let (tx, mut rx) = mpsc::unbounded_channel();
        
        // Spawn task to handle outgoing messages
        tokio::spawn(async move {
            while let Some(msg) = rx.recv().await {
                // Messages are already formatted, just forward them
            }
        });

        let player = Player::new(id, position, tx);
        self.players.insert(id, player);
    }

    pub fn remove_player(&mut self, id: Uuid) {
        self.players.remove(&id);
    }

    pub fn get_player_mut(&mut self, id: Uuid) -> Option<dashmap::mapref::one::RefMut<Uuid, Player>> {
        self.players.get_mut(&id)
    }

    pub fn get_all_players_except(&self, exclude_id: Uuid) -> Vec<PlayerInfo> {
        self.players
            .iter()
            .filter(|entry| *entry.key() != exclude_id)
            .map(|entry| {
                let player = entry.value();
                PlayerInfo {
                    id: player.id.to_string(),
                    position: Position {
                        x: player.position.x,
                        y: player.position.y,
                        z: player.position.z,
                    },
                }
            })
            .collect()
    }

    pub async fn broadcast_except(&self, exclude_id: Uuid, msg: &ServerMessage) {
        for entry in self.players.iter() {
            if *entry.key() != exclude_id {
                entry.value().send_message(msg).await;
            }
        }
    }

    pub async fn broadcast_to_all(&self, msg: &ServerMessage) {
        for entry in self.players.iter() {
            entry.value().send_message(msg).await;
        }
    }

    pub fn update_physics(&mut self, physics: &mut PhysicsWorld) {
        for mut entry in self.players.iter_mut() {
            let player = entry.value_mut();
            
            // Create rigid body if it doesn't exist
            if player.rigid_body_handle.is_none() {
                let (body_handle, collider_handle) = physics.create_player_body(
                    player.position,
                    player.rotation,
                );
                player.rigid_body_handle = Some(body_handle);
                player.collider_handle = Some(collider_handle);
            }
            
            // Update physics body with player state
            if let Some(handle) = player.rigid_body_handle {
                physics.update_player_body(
                    handle,
                    player.position,
                    player.rotation,
                    player.velocity,
                );
            }
        }
    }
}
