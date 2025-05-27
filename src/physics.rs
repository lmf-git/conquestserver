use nalgebra::{Point3, Vector3};
use rapier3d::prelude::*;
use uuid::Uuid;
use std::collections::HashMap;

use crate::player::Player;

pub struct DynamicObject {
    #[allow(dead_code)]  // Used for identification and future cleanup
    pub id: Uuid,
    pub body_handle: RigidBodyHandle,
    #[allow(dead_code)]  // Used for cleanup when removing objects
    pub collider_handle: ColliderHandle,
    pub object_type: String,
    pub scale: f32,
}

pub struct PhysicsWorld {
    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,
    pub physics_pipeline: PhysicsPipeline,
    pub island_manager: IslandManager,
    pub broad_phase: BroadPhase,
    pub narrow_phase: NarrowPhase,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
    pub query_pipeline: QueryPipeline,
    pub players: HashMap<Uuid, Player>,
    pub dynamic_objects: HashMap<Uuid, DynamicObject>,
}

impl PhysicsWorld {
    pub fn new() -> Self {
        let mut physics = Self {
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
            players: HashMap::new(),
            dynamic_objects: HashMap::new(),
        };
        
        // Create game world geometry
        physics.create_planet();
        physics.create_platform();
        physics.create_dynamic_rocks();
        
        physics
    }
    
    pub fn add_player(&mut self, id: Uuid) -> Player {
        let player = Player::new(id, &mut self.rigid_body_set, &mut self.collider_set);
        
        // Log the actual spawn position from physics
        if let Some(body) = self.rigid_body_set.get(player.body_handle) {
            let pos = body.translation();
            tracing::info!(
                "Added player {} to physics world at actual position: [{:.1}, {:.1}, {:.1}]",
                id, pos.x, pos.y, pos.z
            );
        }
        
        self.players.insert(id, player.clone());
        player
    }
    
    pub fn remove_player(&mut self, id: Uuid) {
        tracing::info!("Removing player {} from physics world", id);
        
        if let Some(player) = self.players.get(&id) {
            tracing::debug!("Player {} handles - body: {:?}, collider: {:?}", 
                id, player.body_handle, player.collider_handle);
            
            // Manually remove player's physics bodies with correct Rapier API
            if self.collider_set.contains(player.collider_handle) {
                self.collider_set.remove(
                    player.collider_handle, 
                    &mut self.island_manager,
                    &mut self.rigid_body_set, 
                    true
                );
                tracing::debug!("Removed collider for player {}", id);
            }
            
            if self.rigid_body_set.contains(player.body_handle) {
                self.rigid_body_set.remove(
                    player.body_handle,
                    &mut self.island_manager,
                    &mut self.collider_set,
                    &mut self.impulse_joint_set,
                    &mut self.multibody_joint_set,
                    false
                );
                tracing::debug!("Removed rigid body for player {}", id);
            }
            
            // Remove from our tracking
            self.players.remove(&id);
            
            tracing::info!("Successfully removed player {} from physics world", id);
        } else {
            tracing::warn!("Player {} not found in physics world during removal", id);
        }
    }
    
    fn create_planet(&mut self) {
        // Create a simple sphere for the planet (you can make this more complex later)
        let planet_radius = 200.0;
        let planet_y = -250.0;
        
        let rigid_body = RigidBodyBuilder::fixed()
            .translation(vector![0.0, planet_y, 0.0])
            .build();
        
        let handle = self.rigid_body_set.insert(rigid_body);
        
        let collider = ColliderBuilder::ball(planet_radius)
            .friction(0.8)
            .restitution(0.1)
            .build();
        
        self.collider_set.insert_with_parent(collider, handle, &mut self.rigid_body_set);
    }
    
    fn create_platform(&mut self) {
        // Main platform - ensure it matches client visual
        let platform_half_extents = vector![25.0, 1.5, 25.0]; // 50x3x50 total size
        
        let rigid_body = RigidBodyBuilder::fixed()
            .translation(vector![0.0, 30.0, 0.0]) // Platform center at y=30
            .build();
        
        let handle = self.rigid_body_set.insert(rigid_body);
        
        let collider = ColliderBuilder::cuboid(
            platform_half_extents.x,
            platform_half_extents.y, 
            platform_half_extents.z
        )
        .friction(0.8)
        .restitution(0.2)
        .build();
        
        self.collider_set.insert_with_parent(collider, handle, &mut self.rigid_body_set);
        
        tracing::info!("Created platform at y=30.0 with size 50x3x50");
    }
    
    fn create_dynamic_rocks(&mut self) {
        // Only create 1 rock for now to avoid constraint solver issues
        self.create_pushable_rock(vector![15.0, 40.0, 15.0], 1.0);
        
        tracing::info!("Created minimal dynamic objects to prevent physics solver issues");
    }
    
    fn create_pushable_rock(&mut self, position: Vector3<f32>, scale: f32) {
        let id = Uuid::new_v4();
        
        // Create rock rigid body with ultra-conservative settings
        let rock_body = RigidBodyBuilder::dynamic()
            .translation(position)
            .linear_damping(1.5) // Very high damping
            .angular_damping(1.5) // Very high damping
            .can_sleep(true)
            .build();
        
        let body_handle = self.rigid_body_set.insert(rock_body);
        
        // Create rock collider (sphere for maximum stability)
        let rock_collider = ColliderBuilder::ball(1.0 * scale) // Smaller radius
            .density(0.3) // Very light
            .friction(1.0) // Maximum friction
            .restitution(0.1) // Minimal bouncing
            .build();
        
        let collider_handle = self.collider_set.insert_with_parent(
            rock_collider,
            body_handle,
            &mut self.rigid_body_set,
        );
        
        let dynamic_object = DynamicObject {
            id,
            body_handle,
            collider_handle,
            object_type: "rock".to_string(),
            scale,
        };
        
        self.dynamic_objects.insert(id, dynamic_object);
        
        tracing::info!(
            "Created pushable rock {} at [{:.1}, {:.1}, {:.1}] with scale {:.1}",
            id, position.x, position.y, position.z, scale
        );
    }
    
    pub fn get_dynamic_objects_state(&self) -> HashMap<String, crate::messages::DynamicObjectState> {
        let mut state = HashMap::new();
        
        for (id, obj) in &self.dynamic_objects {
            if let Some(body) = self.rigid_body_set.get(obj.body_handle) {
                let pos = body.translation();
                let vel = body.linvel();
                let rot = body.rotation();
                
                let object_state = crate::messages::DynamicObjectState {
                    position: [pos.x, pos.y, pos.z],
                    velocity: [vel.x, vel.y, vel.z],
                    rotation: [rot.i, rot.j, rot.k, rot.w],
                    object_type: obj.object_type.clone(),
                    scale: obj.scale,
                };
                
                state.insert(id.to_string(), object_state);
            }
        }
        
        state
    }
    
    pub fn step(&mut self) {
        // Apply planet-centered gravity to all dynamic bodies including players
        let planet_center = Point3::new(0.0, -250.0, 0.0);
        let gravity_strength = 9.8; // Reduced from 12.0 to prevent instability
        
        // IMPORTANT: Collect valid player handles first to avoid accessing removed players
        let valid_player_handles: Vec<(Uuid, RigidBodyHandle)> = self.players.iter()
            .filter_map(|(id, player)| {
                if self.rigid_body_set.contains(player.body_handle) {
                    Some((*id, player.body_handle))
                } else {
                    None
                }
            })
            .collect();
        
        // Apply gravity to each dynamic body safely with velocity limits
        for (handle, body) in self.rigid_body_set.iter_mut() {
            if body.body_type() == RigidBodyType::Dynamic {
                let pos = *body.translation();
                let to_planet_vector = planet_center - pos;
                let distance = to_planet_vector.coords.magnitude();
                
                let gravity_multiplier = if distance > 150.0 {
                    f32::min(400.0 / distance, 0.8)
                } else {
                    0.8
                };
                
                let gravity_dir = to_planet_vector.coords.normalize();
                let effective_gravity = gravity_strength * gravity_multiplier;
                
                let current_vel = *body.linvel();
                let gravity_force = gravity_dir * effective_gravity * (1.0 / 60.0); // Fixed timestep
                
                let new_vel = current_vel + gravity_force;
                
                // CRITICAL: Much more aggressive velocity clamping for players
                let is_player = valid_player_handles.iter().any(|(_, ph)| *ph == handle);
                let max_velocity = if is_player { 25.0 } else { 40.0 }; // Lower limits for players
                
                let velocity_magnitude = new_vel.magnitude();
                if velocity_magnitude > max_velocity {
                    let clamped_vel = new_vel.normalize() * max_velocity;
                    body.set_linvel(clamped_vel, true);
                    
                    if is_player {
                        tracing::warn!("Player body has excessive velocity {:.1}, clamping to {:.1}", 
                                      velocity_magnitude, max_velocity);
                    }
                } else {
                    body.set_linvel(new_vel, true);
                }
            }
        }
        
        // Use very conservative integration parameters
        let mut integration_params = IntegrationParameters::default();
        integration_params.dt = 1.0 / 60.0; // Fixed timestep
        integration_params.min_ccd_dt = 1.0 / 240.0; // Very small CCD timestep
        integration_params.max_ccd_substeps = 1; // Minimal CCD substeps
        integration_params.allowed_linear_error = 0.001; // Very tight tolerance
        integration_params.erp = 0.05; // Very low error reduction to prevent instability
        integration_params.damping_ratio = 0.8; // High damping
        integration_params.joint_erp = 0.05; // Very low joint error reduction
        integration_params.joint_damping_ratio = 0.8; // High joint damping
        
        // Before stepping, ensure all rigid bodies are valid and have reasonable values
        let mut invalid_players = Vec::new();
        for (player_id, player) in &self.players {
            if let Some(body) = self.rigid_body_set.get(player.body_handle) {
                let vel = body.linvel();
                let pos = body.translation();
                
                // Check for NaN or extreme values
                if vel.x.is_nan() || vel.y.is_nan() || vel.z.is_nan() ||
                   pos.x.is_nan() || pos.y.is_nan() || pos.z.is_nan() ||
                   vel.magnitude() > 100.0 || pos.magnitude() > 10000.0 {
                    tracing::error!("Player {} has invalid physics state - vel: [{:.1}, {:.1}, {:.1}], pos: [{:.1}, {:.1}, {:.1}]",
                                   player_id, vel.x, vel.y, vel.z, pos.x, pos.y, pos.z);
                    invalid_players.push(*player_id);
                }
            } else {
                tracing::warn!("Player {} has invalid body handle: {:?}", player_id, player.body_handle);
                invalid_players.push(*player_id);
            }
        }
        
        // Remove any players with invalid handles or states
        for player_id in invalid_players {
            tracing::warn!("Removing player {} with invalid state from tracking", player_id);
            self.players.remove(&player_id);
        }
        
        // Step the physics simulation
        self.physics_pipeline.step(
            &vector![0.0, 0.0, 0.0], // No global gravity
            &integration_params,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            None,
            &(),
            &()
        );
        
        // Update query pipeline for raycasting
        self.query_pipeline.update(&self.rigid_body_set, &self.collider_set);
        
        // Update player physics SAFELY - collect player IDs first to avoid borrowing conflicts
        let player_ids: Vec<Uuid> = self.players.keys().cloned().collect();
        for player_id in player_ids {
            if let Some(player) = self.players.get_mut(&player_id) {
                // Get body handle before any borrows
                let body_handle = player.body_handle;
                
                // First check - collect velocity data if body exists
                let velocity_data = if let Some(body) = self.rigid_body_set.get(body_handle) {
                    let vel = body.linvel();
                    let magnitude = vel.magnitude();
                    if magnitude > 25.0 {
                        Some((magnitude, *vel))
                    } else {
                        None
                    }
                } else {
                    None
                };
                
                // Second step - apply emergency velocity reset if needed
                if let Some((magnitude, old_vel)) = velocity_data {
                    if let Some(body_mut) = self.rigid_body_set.get_mut(body_handle) {
                        tracing::error!("Player {} velocity {:.1} exceeded safe limits after physics step, emergency reset", 
                                       player_id, magnitude);
                        let safe_vel = nalgebra::Vector3::new(old_vel.x, old_vel.y, old_vel.z).normalize() * 10.0;
                        body_mut.set_linvel(nalgebra::Vector3::new(safe_vel.x, safe_vel.y, safe_vel.z), true);
                    }
                }
                
                // Third step - update player physics state
                if let Some(body) = self.rigid_body_set.get(body_handle) {
                    player.update_physics(body);
                } else {
                    tracing::warn!("Player {} physics body not found during update", player_id);
                }
            }
        }
    }
}
