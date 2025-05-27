use axum::{
    extract::{
        State,
        ws::{Message, WebSocket, WebSocketUpgrade},
    },
    response::IntoResponse,
    routing::get,
    Router,
};
use dashmap::DashMap;
use futures::{sink::SinkExt, stream::StreamExt};
use std::{net::SocketAddr, sync::Arc, time::Duration};
use tokio::sync::{RwLock, broadcast};
use tower_http::cors::CorsLayer;
use tracing::{error, info};
use uuid::Uuid;
use nalgebra::Point3; // Add this import

mod physics;
mod player;
mod messages;

use physics::PhysicsWorld;
use player::Player;
use messages::{ClientMessage, ServerMessage, PlayerState};

#[derive(Clone)]
struct GameState {
    players: Arc<DashMap<Uuid, Player>>,
    physics_world: Arc<RwLock<PhysicsWorld>>,
    state_broadcaster: broadcast::Sender<ServerMessage>,
}

#[tokio::main]
async fn main() {
    // Initialize tracing
    tracing_subscriber::fmt::init();

    // Create broadcast channel for state updates
    let (state_tx, _) = broadcast::channel::<ServerMessage>(256);

    // Create game state
    let game_state = GameState {
        players: Arc::new(DashMap::new()),
        physics_world: Arc::new(RwLock::new(PhysicsWorld::new())),
        state_broadcaster: state_tx,
    };

    // Start physics simulation loop
    let physics_state = game_state.clone();
    tokio::spawn(async move {
        physics_loop(physics_state).await;
    });

    // Create router
    let app = Router::new()
        .route("/ws", get(websocket_handler))
        .layer(CorsLayer::permissive())
        .with_state(game_state);

    // Start server using tokio::net::TcpListener
    let addr = SocketAddr::from(([127, 0, 0, 1], 8080));
    info!("Game server listening on {}", addr);
    
    let listener = tokio::net::TcpListener::bind(&addr)
        .await
        .expect("Failed to bind to address");
    
    axum::serve(listener, app)
        .await
        .expect("Failed to start server");
}

async fn websocket_handler(
    ws: WebSocketUpgrade,
    State(game_state): State<GameState>,
) -> impl IntoResponse {
    ws.on_upgrade(move |socket| handle_socket(socket, game_state))
}

async fn handle_socket(socket: WebSocket, game_state: GameState) {
    let player_id = Uuid::new_v4();
    let (mut sender, mut receiver) = socket.split();
    
    // Create channels for this player
    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel::<ServerMessage>();
    
    // Add player to game
    {
        let mut physics_world = game_state.physics_world.write().await;
        let player = physics_world.add_player(player_id);
        game_state.players.insert(player_id, player);
    }
    
    // Subscribe to broadcast updates
    let mut broadcast_rx = game_state.state_broadcaster.subscribe();
    
    // IMPORTANT: Send init message FIRST before any physics updates
    let current_state = get_active_players_state_excluding(&game_state, player_id).await;
    let dynamic_objects = get_dynamic_objects_state(&game_state).await;
    let init_msg = ServerMessage::Init {
        player_id: player_id.to_string(),
        state: current_state,
        dynamic_objects,
    };
    
    if let Err(e) = tx.send(init_msg) {
        error!("Failed to send init message: {}", e);
    }
    
    info!("Player {} connected and sent current state with {} other players", player_id, game_state.players.len() - 1);
    
    // Add small delay to ensure client processes init message before receiving state updates
    tokio::time::sleep(Duration::from_millis(100)).await;
    
    // NOW force a physics update and sync
    {
        let mut physics_world = game_state.physics_world.write().await;
        physics_world.step();
        
        // Sync physics world players to game state players
        for (player_id, physics_player) in &physics_world.players {
            if let Some(mut game_player) = game_state.players.get_mut(player_id) {
                // Copy all fields from physics player to game player
                game_player.value_mut().position = physics_player.position;
                game_player.value_mut().velocity = physics_player.velocity;
                game_player.value_mut().rotation = physics_player.rotation;
                game_player.value_mut().is_grounded = physics_player.is_grounded;
                game_player.value_mut().world_origin = physics_player.world_origin;
                game_player.value_mut().input_sequence = physics_player.input_sequence;
            }
        }
    }
    
    // NOW broadcast state to others
    broadcast_state(&game_state).await;
    
    // Track last activity for timeout detection
    let last_activity = Arc::new(RwLock::new(tokio::time::Instant::now()));
    let last_activity_clone = last_activity.clone();
    
    // Spawn task to handle outgoing messages
    let outgoing_task = tokio::spawn(async move {
        let mut timeout_check = tokio::time::interval(Duration::from_secs(5));
        
        loop {
            tokio::select! {
                Some(msg) = rx.recv() => {
                    let msg_text = serde_json::to_string(&msg).unwrap();
                    if sender.send(Message::Text(msg_text)).await.is_err() {
                        info!("Failed to send message to player {}, disconnecting", player_id);
                        break;
                    }
                }
                Ok(broadcast_msg) = broadcast_rx.recv() => {
                    let msg_text = serde_json::to_string(&broadcast_msg).unwrap();
                    if sender.send(Message::Text(msg_text)).await.is_err() {
                        info!("Failed to send broadcast to player {}, disconnecting", player_id);
                        break;
                    }
                }
                _ = timeout_check.tick() => {
                    // Check if player has timed out (no activity for 30 seconds)
                    let last = *last_activity_clone.read().await;
                    if last.elapsed() > Duration::from_secs(30) {
                        info!("Player {} timed out, disconnecting", player_id);
                        break;
                    }
                }
                else => break,
            }
        }
    });
    
    // Handle incoming messages with timeout
    let mut interval = tokio::time::interval(Duration::from_millis(100));
    let mut consecutive_errors = 0;
    
    loop {
        tokio::select! {
            msg = receiver.next() => {
                match msg {
                    Some(Ok(msg)) => {
                        consecutive_errors = 0;
                        *last_activity.write().await = tokio::time::Instant::now();
                        
                        if let Message::Text(text) = msg {
                            if let Ok(client_msg) = serde_json::from_str::<ClientMessage>(&text) {
                                handle_client_message(
                                    player_id,
                                    client_msg,
                                    &game_state,
                                    &tx,
                                ).await;
                            }
                        } else if let Message::Close(_) = msg {
                            info!("Player {} sent close message", player_id);
                            break;
                        }
                    }
                    Some(Err(e)) => {
                        error!("WebSocket error for player {}: {}", player_id, e);
                        consecutive_errors += 1;
                        if consecutive_errors > 3 {
                            info!("Too many errors for player {}, disconnecting", player_id);
                            break;
                        }
                    }
                    None => {
                        info!("WebSocket closed for player {}", player_id);
                        break;
                    }
                }
            }
            _ = interval.tick() => {
                // Check connection health periodically
                if last_activity.read().await.elapsed() > Duration::from_secs(30) {
                    info!("Player {} inactive for too long, disconnecting", player_id);
                    break;
                }
            }
        }
    }
    
    // Clean up when player disconnects
    info!("Cleaning up player {}", player_id);
    outgoing_task.abort();
    
    // Remove player from game with error handling
    {
        let mut physics_world = game_state.physics_world.write().await;
        
        // Remove from game state first
        if let Some(_player) = game_state.players.remove(&player_id) {
            info!("Removed player {} from game state", player_id);
        }
        
        // Then remove from physics world
        physics_world.remove_player(player_id);
        
        // Log remaining player count
        info!("Remaining players in physics world: {}", physics_world.players.len());
    }
    
    // Small delay to ensure physics world has processed the removal
    tokio::time::sleep(Duration::from_millis(50)).await;
    
    // Notify others that player left
    broadcast_player_left(player_id, &game_state).await;
    
    info!("Player {} disconnected and cleaned up", player_id);
}

async fn handle_client_message(
    player_id: Uuid,
    msg: ClientMessage,
    game_state: &GameState,
    tx: &tokio::sync::mpsc::UnboundedSender<ServerMessage>,
) {
    match msg {
        ClientMessage::Input { input, sequence } => {
            // CRITICAL: Rate limit input processing to prevent velocity buildup
            use once_cell::sync::Lazy;
            use std::sync::Mutex;
            
            static LAST_INPUT_TIME: Lazy<Mutex<std::collections::HashMap<Uuid, std::time::Instant>>> = 
                Lazy::new(|| Mutex::new(std::collections::HashMap::new()));
            
            let now = std::time::Instant::now();
            let should_process = {
                let mut last_times = LAST_INPUT_TIME.lock().unwrap();
                let last_time = last_times.get(&player_id).copied().unwrap_or(now - std::time::Duration::from_millis(50));
                
                // Only process input if enough time has passed (minimum 16ms = 60fps)
                if now.duration_since(last_time) >= std::time::Duration::from_millis(16) {
                    last_times.insert(player_id, now);
                    true
                } else {
                    false
                }
            };
            
            if !should_process {
                return; // Skip this input to prevent excessive velocity buildup
            }
            
            // Process input in a more careful way to avoid borrow conflicts
            {
                let mut physics_world = game_state.physics_world.write().await;
                
                // First, apply input to the player
                if let Some(physics_player) = physics_world.players.get_mut(&player_id) {
                    physics_player.apply_input(input, sequence);
                }
                
                // Then, handle movement application without borrowing conflicts
                let player_exists = physics_world.players.contains_key(&player_id);
                if player_exists {
                    // Extract the needed data to avoid multiple borrows
                    let (body_handle, player_input, player_is_grounded) = {
                        let player = physics_world.players.get(&player_id).unwrap();
                        (player.body_handle, player.input.clone(), player.is_grounded)
                    };
                    
                    // Handle movement logic directly to avoid borrow conflicts
                    if let Some(body) = physics_world.rigid_body_set.get_mut(body_handle) {
                        // Apply the movement logic directly
                        let current_vel = body.linvel();
                        let current_speed = current_vel.magnitude();
                        
                        if current_speed <= 15.0 { // Only apply movement if speed is reasonable
                            // Simple movement application
                            let mut move_forward = 0.0f32;
                            let mut move_right = 0.0f32;
                            
                            if player_input.forward { move_forward += 1.0; }
                            if player_input.backward { move_forward -= 1.0; }
                            if player_input.left { move_right -= 1.0; }
                            if player_input.right { move_right += 1.0; }
                            
                            let move_length = (move_forward * move_forward + move_right * move_right).sqrt();
                            if move_length > 0.0 {
                                move_forward /= move_length;
                                move_right /= move_length;
                            }
                            
                            let base_speed = if player_input.run { 8.0 } else { 5.0 };
                            move_forward *= base_speed;
                            move_right *= base_speed;
                            
                            let rotation = body.rotation();
                            let forward = rotation.transform_vector(&nalgebra::Vector3::new(0.0, 0.0, -1.0));
                            let right = rotation.transform_vector(&nalgebra::Vector3::new(1.0, 0.0, 0.0));
                            
                            let movement_dir = forward * move_forward + right * move_right;
                            
                            if player_is_grounded {
                                let desired_horizontal = movement_dir * base_speed;
                                let vertical_component = current_vel.y;
                                
                                let new_velocity = if move_length > 0.1 {
                                    nalgebra::Vector3::new(desired_horizontal.x, vertical_component, desired_horizontal.z)
                                } else {
                                    nalgebra::Vector3::new(current_vel.x * 0.8, vertical_component, current_vel.z * 0.8)
                                };
                                
                                let final_velocity = if player_input.jump && player_is_grounded {
                                    nalgebra::Vector3::new(new_velocity.x, new_velocity.y + 5.0, new_velocity.z)
                                } else {
                                    new_velocity
                                };
                                
                                body.set_linvel(final_velocity, true);
                            }
                        } else {
                            // Emergency brake
                            let safe_vel = current_vel.normalize() * 10.0;
                            body.set_linvel(safe_vel, true);
                        }
                    }
                }
                
                // Update the player's stored state in a completely separate scope
                if physics_world.players.contains_key(&player_id) {
                    let body_handle = physics_world.players.get(&player_id).unwrap().body_handle;
                    
                    // Get the body data we need first
                    let body_data = {
                        if let Some(body) = physics_world.rigid_body_set.get(body_handle) {
                            let translation = body.translation();
                            let velocity = body.linvel();
                            let rotation = body.rotation();
                            Some((
                                Point3::new(translation.x, translation.y, translation.z),
                                nalgebra::Vector3::new(velocity.x, velocity.y, velocity.z),
                                *rotation
                            ))
                        } else {
                            None // Body doesn't exist
                        }
                    };
                    
                    // Now update the player with this data if we got it
                    if let Some((body_translation, body_velocity, body_rotation)) = body_data {
                        if let Some(physics_player_mut) = physics_world.players.get_mut(&player_id) {
                            physics_player_mut.position = body_translation;
                            physics_player_mut.velocity = body_velocity;
                            physics_player_mut.rotation = body_rotation;
                            
                            // Velocity sanity check
                            let vel_magnitude = body_velocity.magnitude();
                            if vel_magnitude > 30.0 {
                                tracing::warn!("Player {} has high velocity during update: {:.1}", player_id, vel_magnitude);
                            }
                        }
                    }
                }
                
                // Log excessive velocity
                if let Some(player) = physics_world.players.get(&player_id) {
                    if let Some(body) = physics_world.rigid_body_set.get(player.body_handle) {
                        let vel = body.linvel();
                        let speed = vel.magnitude();
                        if speed > 15.0 {
                            tracing::warn!("Player {} has high velocity {:.1} after input processing", player_id, speed);
                        }
                    }
                }
            }
        }
        ClientMessage::Ping { timestamp } => {
            let pong = ServerMessage::Pong {
                timestamp,
            };
            let _ = tx.send(pong);
        }
    }
}

async fn physics_loop(game_state: GameState) {
    let mut interval = tokio::time::interval(Duration::from_millis(16)); // 60 FPS
    let mut last_broadcast = tokio::time::Instant::now();
    let mut frame_count = 0u64;
    
    loop {
        interval.tick().await;
        frame_count += 1;
        
        // Update physics - use async write instead of blocking_write
        {
            let mut physics_world = game_state.physics_world.write().await;
            physics_world.step();
            
            // CRITICAL: Sync physics world players back to game state after physics step
            // This ensures broadcast_state gets the updated positions
            for (player_id, physics_player) in &physics_world.players {
                if let Some(mut game_player) = game_state.players.get_mut(player_id) {
                    // Get current physics body state
                    if let Some(body) = physics_world.rigid_body_set.get(physics_player.body_handle) {
                        let pos = body.translation();
                        let vel = body.linvel();
                        let rot = body.rotation();
                        
                        // Update game player with physics state
                        game_player.position = nalgebra::Point3::new(pos.x, pos.y, pos.z);
                        game_player.velocity = nalgebra::Vector3::new(vel.x, vel.y, vel.z);
                        game_player.rotation = *rot; // rot is already a UnitQuaternion
                        game_player.is_grounded = physics_player.is_grounded;
                        game_player.world_origin = physics_player.world_origin;
                        game_player.input_sequence = physics_player.input_sequence;
                    }
                }
            }
        }
        
        // Log player states every 3 seconds (180 frames) for debugging movement
        if frame_count % 180 == 0 {
            let physics_world = game_state.physics_world.read().await;
            for (player_id, player) in &physics_world.players {
                // Get actual physics body position
                if let Some(body) = physics_world.rigid_body_set.get(player.body_handle) {
                    let physics_pos = body.translation();
                    let physics_vel = body.linvel();
                    let world_pos = Point3::new(physics_pos.x, physics_pos.y, physics_pos.z) + player.world_origin;
                    let is_moving = physics_vel.magnitude() > 0.1;
                    let has_input = player.input.forward || player.input.backward || 
                                   player.input.left || player.input.right;
                    
                    tracing::info!(
                        "Player {} - Physics pos: [{:.1}, {:.1}, {:.1}], World pos: [{:.1}, {:.1}, {:.1}], Vel: [{:.1}, {:.1}, {:.1}], Grounded: {}, Moving: {}, Has input: {}",
                        player_id,
                        physics_pos.x, physics_pos.y, physics_pos.z,
                        world_pos.x, world_pos.y, world_pos.z,
                        physics_vel.x, physics_vel.y, physics_vel.z,
                        player.is_grounded, is_moving, has_input
                    );
                }
            }
        }
        
        // Broadcast state more frequently to ensure movement is visible
        if last_broadcast.elapsed() > Duration::from_millis(50) { // 20 FPS broadcast rate
            broadcast_state(&game_state).await;
            last_broadcast = tokio::time::Instant::now();
        }
    }
}

// Add new function to get only active players
async fn get_active_players_state_excluding(game_state: &GameState, exclude_player_id: Uuid) -> std::collections::HashMap<String, PlayerState> {
    let mut state = std::collections::HashMap::new();
    
    let physics_world = game_state.physics_world.read().await;
    
    // Only include players that exist in BOTH physics world AND game state, EXCLUDING the specified player
    for entry in game_state.players.iter() {
        let player_id = entry.key();
        
        // Skip the excluded player
        if *player_id == exclude_player_id {
            continue;
        }
        
        if let Some(physics_player) = physics_world.players.get(player_id) {
            // Get CURRENT position from physics body, not stored position
            if let Some(body) = physics_world.rigid_body_set.get(physics_player.body_handle) {
                let current_translation = body.translation();
                let current_velocity = body.linvel();
                let current_rotation = body.rotation();
                
                // CRITICAL: Convert physics body position (local) to world position properly
                let local_pos = Point3::new(current_translation.x, current_translation.y, current_translation.z);
                let world_pos = local_pos + physics_player.world_origin;
                
                let player_state = PlayerState {
                    position: [world_pos.x, world_pos.y, world_pos.z], // Send world position
                    velocity: [current_velocity.x, current_velocity.y, current_velocity.z], // Velocity is same in both coordinate systems
                    rotation: [current_rotation.i, current_rotation.j, current_rotation.k, current_rotation.w],
                    is_grounded: physics_player.is_grounded,
                    input_sequence: physics_player.input_sequence,
                    world_origin: [physics_player.world_origin.x, physics_player.world_origin.y, physics_player.world_origin.z], // Send this player's world origin
                };
                
                state.insert(player_id.to_string(), player_state);
            } else {
                tracing::warn!("Player {} has no physics body, skipping", player_id);
            }
        } else {
            tracing::warn!("Player {} exists in game state but not physics world, skipping", player_id);
        }
    }
    
    tracing::info!("Returning state for {} other players (excluding {})", state.len(), exclude_player_id);
    state
}

// Add function to get all active players (without exclusion)
async fn get_active_players_state(game_state: &GameState) -> std::collections::HashMap<String, PlayerState> {
    let mut state = std::collections::HashMap::new();
    
    let physics_world = game_state.physics_world.read().await;
    
    // Include all players that exist in BOTH physics world AND game state
    for entry in game_state.players.iter() {
        let player_id = entry.key();
        
        if let Some(physics_player) = physics_world.players.get(player_id) {
            // Get CURRENT position from physics body, not stored position
            if let Some(body) = physics_world.rigid_body_set.get(physics_player.body_handle) {
                let current_translation = body.translation();
                let current_velocity = body.linvel();
                let current_rotation = body.rotation();
                
                // CRITICAL: Convert physics body position (local) to world position properly
                let local_pos = Point3::new(current_translation.x, current_translation.y, current_translation.z);
                let world_pos = local_pos + physics_player.world_origin;
                
                let player_state = PlayerState {
                    position: [world_pos.x, world_pos.y, world_pos.z], // Send world position
                    velocity: [current_velocity.x, current_velocity.y, current_velocity.z], // Velocity is same in both coordinate systems
                    rotation: [current_rotation.i, current_rotation.j, current_rotation.k, current_rotation.w],
                    is_grounded: physics_player.is_grounded,
                    input_sequence: physics_player.input_sequence,
                    world_origin: [physics_player.world_origin.x, physics_player.world_origin.y, physics_player.world_origin.z], // Send this player's world origin
                };
                
                state.insert(player_id.to_string(), player_state);
            } else {
                tracing::warn!("Player {} has no physics body, skipping", player_id);
            }
        } else {
            tracing::warn!("Player {} exists in game state but not physics world, skipping", player_id);
        }
    }
    
    state
}

// Add function to get dynamic objects state
async fn get_dynamic_objects_state(game_state: &GameState) -> std::collections::HashMap<String, crate::messages::DynamicObjectState> {
    let physics_world = game_state.physics_world.read().await;
    physics_world.get_dynamic_objects_state()
}

// Update broadcast_state to use the new function
async fn broadcast_state(game_state: &GameState) {
    let state = get_active_players_state(game_state).await;
    let dynamic_objects = get_dynamic_objects_state(game_state).await;
    let msg = ServerMessage::State { state, dynamic_objects };
    
    // Broadcast to all connected clients
    let _ = game_state.state_broadcaster.send(msg);
}

async fn broadcast_player_left(player_id: Uuid, game_state: &GameState) {
    let msg = ServerMessage::PlayerLeft {
        player_id: player_id.to_string(),
    };
    
    let _ = game_state.state_broadcaster.send(msg);
}
