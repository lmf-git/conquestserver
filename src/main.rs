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
    
    // Force a physics update to ensure all positions are current
    {
        let mut physics_world = game_state.physics_world.write().await;
        physics_world.step();
        
        // Sync physics world players to game state players immediately
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
    
    // Send initial state to new player with current positions from physics world
    let current_state = get_game_state(&game_state).await;
    let dynamic_objects = get_dynamic_objects_state(&game_state).await;
    let init_msg = ServerMessage::Init {
        player_id: player_id.to_string(),
        state: current_state,
        dynamic_objects,
    };
    
    if let Err(e) = tx.send(init_msg) {
        error!("Failed to send init message: {}", e);
    }
    
    info!("Player {} connected and sent current state with {} players", player_id, game_state.players.len());
    
    // IMPORTANT: Broadcast updated state to all existing players so they see the new player
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
            // Update ONLY in physics world - this is the authoritative source
            {
                let mut physics_world = game_state.physics_world.write().await;
                if let Some(physics_player) = physics_world.players.get_mut(&player_id) {
                    // Apply input to physics player (this will update their movement)
                    physics_player.apply_input(input, sequence);
                    
                    // Log input received for debugging - but less frequently
                    if sequence % 120 == 0 { // Log every 2 seconds instead of every second
                        tracing::debug!(
                            "Applied input for player {} - sequence: {}, forward: {}, yaw: {:.2}",
                            player_id, sequence, physics_player.input.forward, physics_player.input.yaw
                        );
                    }
                }
                
                // Don't call update_physics here - let the main physics loop handle it
                // This avoids borrowing conflicts and ensures consistent physics timing
            }
            // Note: Don't update game_state.players here - let physics loop sync it
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
        
        // Update physics
        {
            let mut physics_world = game_state.physics_world.write().await;
            physics_world.step();
            
            // Log player states every 3 seconds (180 frames) for debugging movement
            if frame_count % 180 == 0 {
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
        }
        
        // Broadcast state more frequently to ensure movement is visible
        if last_broadcast.elapsed() > Duration::from_millis(50) { // 20 FPS broadcast rate
            broadcast_state(&game_state).await;
            last_broadcast = tokio::time::Instant::now();
        }
    }
}

// Get state directly from physics world which is authoritative
async fn get_game_state(game_state: &GameState) -> std::collections::HashMap<String, PlayerState> {
    let mut state = std::collections::HashMap::new();
    
    let physics_world = game_state.physics_world.read().await;
    
    for (player_id, physics_player) in &physics_world.players {
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
            
            // Log coordinate conversion for debugging movement
            let is_moving = current_velocity.magnitude() > 0.1;
            
            if is_moving {
                static MOVEMENT_LOG_COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
                let log_count = MOVEMENT_LOG_COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                
                if log_count % 60 == 0 {
                    tracing::debug!(
                        "Broadcasting player {}: local pos=[{:.1}, {:.1}, {:.1}], world origin=[{:.1}, {:.1}, {:.1}], world pos=[{:.1}, {:.1}, {:.1}], vel=[{:.1}, {:.1}, {:.1}]",
                        player_id,
                        local_pos.x, local_pos.y, local_pos.z,
                        physics_player.world_origin.x, physics_player.world_origin.y, physics_player.world_origin.z,
                        world_pos.x, world_pos.y, world_pos.z,
                        current_velocity.x, current_velocity.y, current_velocity.z
                    );
                }
            }
            
            state.insert(player_id.to_string(), player_state);
        } else {
            tracing::warn!("Player {} physics body not found when getting state", player_id);
        }
    }
    
    state
}

async fn get_dynamic_objects_state(game_state: &GameState) -> std::collections::HashMap<String, messages::DynamicObjectState> {
    let physics_world = game_state.physics_world.read().await;
    physics_world.get_dynamic_objects_state()
}

async fn broadcast_state(game_state: &GameState) {
    let state = get_game_state(game_state).await;
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
