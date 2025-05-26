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
    pub integration_parameters: IntegrationParameters,
    pub physics_pipeline: PhysicsPipeline,
    pub island_manager: IslandManager,
    pub broad_phase: BroadPhase,
    pub narrow_phase: NarrowPhase,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
    pub query_pipeline: QueryPipeline,
    pub gravity: Vector3<Real>,
    pub players: HashMap<Uuid, Player>,
    pub dynamic_objects: HashMap<Uuid, DynamicObject>,
}

impl PhysicsWorld {
    pub fn new() -> Self {
        // No global gravity - we'll apply planet-centered gravity per object
        let gravity = Vector3::new(0.0, 0.0, 0.0);
        
        let mut physics = Self {
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            integration_parameters: IntegrationParameters::default(),
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
            gravity,
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
            
            // Use the player's own removal method
            player.remove_from_world(&mut self.rigid_body_set, &mut self.collider_set);
            
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
                
                state.insert(
                    id.to_string(),
                    crate::messages::DynamicObjectState {
                        position: [pos.x, pos.y, pos.z],
                        velocity: [vel.x, vel.y, vel.z],
                        rotation: [rot.i, rot.j, rot.k, rot.w],
                        object_type: obj.object_type.clone(),
                        scale: obj.scale,
                    }
                );
            }
        }
        
        state
    }
    
    pub fn step(&mut self) {
        // Apply planet-centered gravity to all dynamic bodies including players
        let planet_center = Point3::new(0.0, -250.0, 0.0);
        let gravity_strength = 12.0; // Further reduced gravity strength to prevent instability
        
        // Collect body handles SAFELY to avoid borrowing issues
        let mut body_handles: Vec<RigidBodyHandle> = Vec::new();
        for (handle, body) in self.rigid_body_set.iter() {
            if body.body_type() == RigidBodyType::Dynamic {
                body_handles.push(handle);
            }
        }
        
        // Apply gravity to each dynamic body safely
        for handle in body_handles {
            // Check if body still exists before accessing it
            if let Some(body) = self.rigid_body_set.get_mut(handle) {
                let position = body.translation();
                
                // Check if this is a player body and get their world origin
                let world_origin = if let Some(player) = self.players.values()
                    .find(|p| p.body_handle == handle) {
                    player.world_origin
                } else {
                    Vector3::zeros()
                };
                
                // Calculate world position for gravity calculation
                let world_position = Vector3::new(position.x, position.y, position.z) + world_origin;
                
                // Calculate direction from object to planet center
                let to_planet = planet_center - Point3::from(world_position);
                let distance_to_planet = to_planet.magnitude();
                
                // Apply very conservative gravity scaling
                let gravity_multiplier = if distance_to_planet > 150.0 {
                    (400.0 / distance_to_planet).min(0.8) // Even more conservative
                } else {
                    0.8 // Reduced constant close-range gravity
                };
                
                let gravity_dir = to_planet.normalize();
                let effective_gravity = gravity_strength * gravity_multiplier;
                let gravity_force = gravity_dir * effective_gravity * body.mass();
                
                body.add_force(gravity_force, true);
            }
        }
        
        // Use extremely conservative integration parameters to prevent solver crashes
        let mut ultra_conservative_params = self.integration_parameters.clone();
        ultra_conservative_params.dt = 1.0 / 60.0; // Fixed timestep
        ultra_conservative_params.max_velocity_iterations = 1; // Minimal iterations
        ultra_conservative_params.max_velocity_friction_iterations = 1; // Minimal friction iterations
        ultra_conservative_params.max_stabilization_iterations = 1; // Minimal stabilization
        ultra_conservative_params.max_ccd_substeps = 1; // Minimal CCD
        ultra_conservative_params.erp = 0.2; // Reduced error reduction parameter
        ultra_conservative_params.damping_ratio = 0.25; // Add some damping
        ultra_conservative_params.joint_erp = 0.2; // Reduced joint error reduction
        ultra_conservative_params.joint_damping_ratio = 0.25; // Add joint damping
        
        // Step the physics simulation with ultra-conservative parameters
        self.physics_pipeline.step(
            &self.gravity, // This is still (0,0,0) - we handle gravity manually above
            &ultra_conservative_params,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            Some(&mut self.query_pipeline),
            &(),
            &(),
        );
        
        // Update query pipeline for raycasting
        self.query_pipeline.update(&self.rigid_body_set, &self.collider_set);
        
        // Update player physics SAFELY - collect player IDs first to avoid borrowing conflicts
        let player_ids: Vec<Uuid> = self.players.keys().cloned().collect();
        for player_id in player_ids {
            // Verify player and their physics bodies still exist
            if let Some(player) = self.players.get(&player_id) {
                let body_exists = self.rigid_body_set.get(player.body_handle).is_some();
                let collider_exists = self.collider_set.get(player.collider_handle).is_some();
                
                if body_exists && collider_exists {
                    // Safe to update physics
                    if let Some(player_mut) = self.players.get_mut(&player_id) {
                        player_mut.update_physics(&mut self.rigid_body_set, &self.collider_set);
                    }
                } else {
                    tracing::warn!("Player {} physics bodies missing during update, removing from tracking", player_id);
                    self.players.remove(&player_id);
                }
            }
        }
    }
}
