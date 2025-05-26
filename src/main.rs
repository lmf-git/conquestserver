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
    }
    
    // Send initial state to new player with current positions
    let current_state = get_game_state(&game_state).await;
    let init_msg = ServerMessage::Init {
        player_id: player_id.to_string(),
        state: current_state,
    };
    
    if let Err(e) = tx.send(init_msg) {
        error!("Failed to send init message: {}", e);
    }
    
    info!("Player {} connected", player_id);
    
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
    
    // Remove player from game
    {
        let mut physics_world = game_state.physics_world.write().await;
        physics_world.remove_player(player_id);
        game_state.players.remove(&player_id);
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
            // Update in physics world
            {
                let mut physics_world = game_state.physics_world.write().await;
                if let Some(physics_player) = physics_world.players.get_mut(&player_id) {
                    physics_player.apply_input(input.clone(), sequence);
                    
                    // Also update in game state for immediate reflection
                    if let Some(mut game_player) = game_state.players.get_mut(&player_id) {
                        game_player.apply_input(input, sequence);
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
        
        // Update physics
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
            
            // Log player states every second (60 frames)
            if frame_count % 60 == 0 {
                for (player_id, player) in &physics_world.players {
                    let world_pos = player.position + player.world_origin;
                    tracing::debug!(
                        "Player {} - World pos: [{:.1}, {:.1}, {:.1}], Vel: [{:.1}, {:.1}, {:.1}], Grounded: {}",
                        player_id,
                        world_pos.x, world_pos.y, world_pos.z,
                        player.velocity.x, player.velocity.y, player.velocity.z,
                        player.is_grounded
                    );
                }
            }
        }
        
        // Broadcast state update at 20Hz to reduce network traffic
        let now = tokio::time::Instant::now();
        if now.duration_since(last_broadcast) >= Duration::from_millis(50) {
            broadcast_state(&game_state).await;
            last_broadcast = now;
        }
    }
}

// Get state directly from physics world which is authoritative
async fn get_game_state(game_state: &GameState) -> std::collections::HashMap<String, PlayerState> {
    let mut state = std::collections::HashMap::new();
    
    let physics_world = game_state.physics_world.read().await;
    
    for (player_id, physics_player) in &physics_world.players {
        let player_state = physics_player.get_state();
        
        // Debug log before moving the state
        tracing::debug!(
            "Sending state for player {}: pos=[{:.1}, {:.1}, {:.1}], origin=[{:.1}, {:.1}, {:.1}]",
            player_id,
            player_state.position[0],
            player_state.position[1], 
            player_state.position[2],
            player_state.world_origin[0],
            player_state.world_origin[1],
            player_state.world_origin[2]
        );
        
        state.insert(player_id.to_string(), player_state);
    }
    
    state
}

async fn broadcast_state(game_state: &GameState) {
    let state = get_game_state(game_state).await;
    let msg = ServerMessage::State { state };
    
    // Broadcast to all connected clients
    let _ = game_state.state_broadcaster.send(msg);
}

async fn broadcast_player_left(player_id: Uuid, game_state: &GameState) {
    let msg = ServerMessage::PlayerLeft {
        player_id: player_id.to_string(),
    };
    
    let _ = game_state.state_broadcaster.send(msg);
}
