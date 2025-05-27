use nalgebra::{Point3, UnitQuaternion, Vector3, Unit};
use rapier3d::prelude::*;
use uuid::Uuid;

use crate::messages::PlayerInput;

#[derive(Debug, Clone)]
pub struct Player {
    #[allow(dead_code)]  // We use this for logging and identification
    pub id: Uuid,
    pub body_handle: RigidBodyHandle,
    pub collider_handle: ColliderHandle,
    pub input_sequence: u32,
    pub world_origin: Vector3<f32>,
    pub is_grounded: bool,
    // Add missing fields that are referenced in update_physics
    pub position: Point3<f32>,
    pub velocity: Vector3<f32>,
    pub rotation: UnitQuaternion<f32>,
    pub input: PlayerInput,
    pub last_ground_normal: Vector3<f32>,
    // Remove unused collision tracking fields - we use simpler raycast approach
    pub last_ground_contact: f32, // Store as seconds since game start
    pub player_height: f32, // Add this field for grounding calculations
}

impl Player {
    pub fn new(id: Uuid, rigid_body_set: &mut RigidBodySet, collider_set: &mut ColliderSet) -> Self {
        // Randomize spawn position to avoid overlap with other players
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};
        
        let mut hasher = DefaultHasher::new();
        id.hash(&mut hasher);
        let hash = hasher.finish();
        
        // Spread players around a circle on the platform with MORE spacing
        let angle = (hash as f32 / u64::MAX as f32) * 2.0 * std::f32::consts::PI;
        let radius = 8.0; // Increased radius to avoid overlaps
        let spawn_x = angle.cos() * radius;
        let spawn_z = angle.sin() * radius;
        let spawn_y = 40.0; // Higher spawn to ensure they're above platform
        
        // Add small random offset to prevent exact overlaps
        let offset_x = ((hash % 100) as f32 / 100.0 - 0.5) * 2.0;
        let offset_z = (((hash >> 8) % 100) as f32 / 100.0 - 0.5) * 2.0;
        
        let spawn_position = vector![spawn_x + offset_x, spawn_y, spawn_z + offset_z];
        
        tracing::info!("Creating player {} at position: [{:.1}, {:.1}, {:.1}]", 
            id, spawn_position.x, spawn_position.y, spawn_position.z);
        
        // Create player rigid body - DYNAMIC for proper gravity application but with locked rotations
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(spawn_position)
            .lock_rotations() // Lock rotations to prevent tumbling
            .linear_damping(0.5) // Add damping for stability
            .angular_damping(5.0) // High angular damping
            .can_sleep(false) // Keep awake for responsiveness
            .ccd_enabled(false) // Disable CCD to prevent solver issues
            .build();
        
        let body_handle = rigid_body_set.insert(rigid_body);
        
        // Create player capsule collider with more conservative settings
        let player_height = 1.8;
        let player_radius = 0.4;
        
        let collider = ColliderBuilder::capsule_y(
            player_height / 2.0 - player_radius,
            player_radius
        )
        .friction(0.3) // More friction for stability
        .restitution(0.0) // No bouncing
        .density(0.8) // Lighter for less impact forces
        .active_collision_types(ActiveCollisionTypes::default())
        .active_events(ActiveEvents::COLLISION_EVENTS)
        .contact_force_event_threshold(0.0) // Disable contact force events to reduce overhead
        .build();
        
        let collider_handle = collider_set.insert_with_parent(
            collider,
            body_handle,
            rigid_body_set
        );
        
        Self {
            id,
            body_handle,
            collider_handle,
            input_sequence: 0,
            world_origin: Vector3::zeros(),
            is_grounded: false,
            position: Point3::new(spawn_position.x, spawn_position.y, spawn_position.z),
            velocity: Vector3::zeros(),
            rotation: UnitQuaternion::identity(),
            input: PlayerInput {
                forward: false,
                backward: false,
                left: false,
                right: false,
                jump: false,
                run: false,
                yaw: 0.0,
                pitch: 0.0,
                world_position: [spawn_position.x, spawn_position.y, spawn_position.z],
                world_origin: [0.0, 0.0, 0.0],
            },
            last_ground_normal: Vector3::y(),
            last_ground_contact: 0.0,
            player_height,
        }
    }
    
    fn check_grounded(&mut self, rigid_body_set: &RigidBodySet, collider_set: &ColliderSet) {
        // Use the same logic as original client: collisions + raycasts + velocity
        if let Some(body) = rigid_body_set.get(self.body_handle) {
            let body_position = body.translation();
            let local_pos = Point3::new(body_position.x, body_position.y, body_position.z);
            
            // Calculate world position for gravity direction
            let world_pos = local_pos + self.world_origin;
            let ray_origin = Point3::from(world_pos);
            
            // Use same planet center as physics world
            let planet_center = Point3::new(0.0, -250.0, 0.0);
            let to_planet = planet_center - ray_origin;
            let ray_dir = to_planet.normalize();
            let max_distance = 1.5; // Match client raycast distance
            
            // Convert back to local space for raycasting
            let local_ray_origin = Point3::from(local_pos.coords);
            let _ray = Ray::new(local_ray_origin, ray_dir); // Prefix with underscore to indicate intentional
            let filter = QueryFilter::default().exclude_collider(self.collider_handle);
            
            // Create a temporary query pipeline for raycasting
            let mut query_pipeline = QueryPipeline::new();
            query_pipeline.update(rigid_body_set, collider_set);
            
            // Check velocity in gravity direction (same as client)
            let velocity = body.linvel();
            let velocity_in_gravity_dir = velocity.dot(&ray_dir);
            let low_downward_velocity = velocity_in_gravity_dir < 3.0; // Match client threshold
            
            // Check for ray hits (multiple rays like client)
            let player_quat = body.rotation();
            let foot_offset = 0.4 * 0.8; // playerRadius * 0.8
            let foot_level = -self.player_height * 0.5;
            
            // Calculate foot positions like client
            let left_offset = Vector3::new(-foot_offset, foot_level, 0.0);
            let right_offset = Vector3::new(foot_offset, foot_level, 0.0);
            let center_offset = Vector3::new(0.0, foot_level, 0.0);
            
            // Rotate offsets by player rotation
            let left_offset_rotated = player_quat * left_offset;
            let right_offset_rotated = player_quat * right_offset;
            let center_offset_rotated = player_quat * center_offset;
            
            let left_foot_world = local_pos + left_offset_rotated;
            let right_foot_world = local_pos + right_offset_rotated;
            let center_foot_world = local_pos + center_offset_rotated;
            
            // Cast multiple rays like client
            let left_ray = Ray::new(Point3::from(left_foot_world.coords), ray_dir);
            let right_ray = Ray::new(Point3::from(right_foot_world.coords), ray_dir);
            let center_ray = Ray::new(Point3::from(center_foot_world.coords), ray_dir);
            
            let left_hit = query_pipeline.cast_ray(
                rigid_body_set, collider_set, &left_ray, max_distance, true, filter
            );
            let right_hit = query_pipeline.cast_ray(
                rigid_body_set, collider_set, &right_ray, max_distance, true, filter
            );
            let center_hit = query_pipeline.cast_ray(
                rigid_body_set, collider_set, &center_ray, max_distance, true, filter
            );
            
            let has_ray_hits = left_hit.is_some() || right_hit.is_some() || center_hit.is_some();
            
            // Simplified collision detection - just check if we have recent ground contact from raycasts
            let has_ground_collisions = has_ray_hits && self.last_ground_contact > 0.5;
            let recent_ground_contact = self.last_ground_contact > 0.5;
            
            // Use EXACT same grounding logic as client
            self.is_grounded = (has_ground_collisions && low_downward_velocity) || 
                              (has_ray_hits && low_downward_velocity) ||
                              (recent_ground_contact && velocity_in_gravity_dir.abs() < 0.5);
            
            // Store the gravity direction as the ground normal (inverted)
            if self.is_grounded {
                self.last_ground_normal = -ray_dir;
                
                tracing::debug!(
                    "Player {} grounded - Ray hits: {}, Vel in gravity dir: {:.2}",
                    self.id, has_ray_hits, velocity_in_gravity_dir
                );
            }
            
            // Update last ground contact timer
            if has_ray_hits {
                self.last_ground_contact = 1.0; // Mark as recent contact
            } else if self.last_ground_contact > 0.0 {
                self.last_ground_contact -= 0.016; // Approximate frame time
                if self.last_ground_contact < 0.0 {
                    self.last_ground_contact = 0.0;
                }
            }
        }
    }
    
    pub fn apply_input(&mut self, input: PlayerInput, sequence: u32) {
        self.input_sequence = sequence;
        
        // Extract world position and world origin from client
        let client_world_pos = Vector3::new(
            input.world_position[0],
            input.world_position[1],
            input.world_position[2]
        );
        
        let client_world_origin = Vector3::new(
            input.world_origin[0],
            input.world_origin[1], 
            input.world_origin[2]
        );
        
        // CRITICAL: Handle world origin shifts properly
        // If client's world origin is different from ours, we need to reconcile
        let origin_delta = client_world_origin - self.world_origin;
        
        if origin_delta.magnitude() > 1.0 { // Significant origin shift
            tracing::info!(
                "Player {} world origin shift detected: client=[{:.1}, {:.1}, {:.1}], server=[{:.1}, {:.1}, {:.1}], delta=[{:.1}, {:.1}, {:.1}]",
                self.id,
                client_world_origin.x, client_world_origin.y, client_world_origin.z,
                self.world_origin.x, self.world_origin.y, self.world_origin.z,
                origin_delta.x, origin_delta.y, origin_delta.z
            );
            
            // Update our world origin to match client
            self.world_origin = client_world_origin;
            
            // Convert client's world position to new local position
            let new_local_pos = client_world_pos - self.world_origin;
            self.position = Point3::from(new_local_pos);
        } else {
            // Normal case: convert client world position to our local position
            let local_pos = client_world_pos - self.world_origin;
            self.position = Point3::from(local_pos);
        }
        
        // Store input for movement calculation
        self.input = input;
        
        // Log significant input changes for debugging
        if self.input.forward || self.input.backward || self.input.left || self.input.right || 
           self.input.jump || self.input.yaw.abs() > 0.01 {
            tracing::debug!(
                "Player {} input applied - forward: {}, yaw: {:.2}, jump: {}, client world pos: [{:.1}, {:.1}, {:.1}], client origin: [{:.1}, {:.1}, {:.1}], our local pos: [{:.1}, {:.1}, {:.1}]",
                self.id, self.input.forward, self.input.yaw, self.input.jump,
                client_world_pos.x, client_world_pos.y, client_world_pos.z,
                client_world_origin.x, client_world_origin.y, client_world_origin.z,
                self.position.x, self.position.y, self.position.z
            );
        }
    }

    pub fn update_physics(&mut self, rigid_body_set: &mut RigidBodySet, collider_set: &ColliderSet) {
        // First, check grounding using collision events + raycasts (like client)
        self.check_grounded(rigid_body_set, collider_set);
        
        // Then update physics with mutable access
        if let Some(body) = rigid_body_set.get_mut(self.body_handle) {
            // Update position and velocity from physics body FIRST
            let translation = body.translation();
            let linvel = body.linvel();
            let rotation = body.rotation();
            
            self.position = Point3::new(translation.x, translation.y, translation.z);
            self.velocity = Vector3::new(linvel.x, linvel.y, linvel.z);
            self.rotation = *rotation;
            
            // Only apply movement if we have actual input to prevent jitter
            let has_movement_input = self.input.forward || self.input.backward || 
                                   self.input.left || self.input.right;
            
            if has_movement_input {
                // Apply movement based on input
                let current_vel = body.linvel();
                let mut new_velocity = Vector3::new(current_vel.x, current_vel.y, current_vel.z);
                
                // Calculate movement direction
                let mut move_dir: Vector3<f32> = Vector3::zeros();
                
                if self.input.forward {
                    move_dir.z -= 1.0;
                }
                if self.input.backward {
                    move_dir.z += 1.0;
                }
                if self.input.left {
                    move_dir.x -= 1.0;
                }
                if self.input.right {
                    move_dir.x += 1.0;
                }
                
                // Normalize movement
                if move_dir.magnitude() > 0.0 {
                    move_dir = move_dir.normalize();
                }
                
                // Get player rotation for movement direction
                let forward = Vector3::new(0.0, 0.0, -1.0);
                let right = Vector3::new(1.0, 0.0, 0.0);
                let rotated_forward = self.rotation * forward;
                let rotated_right = self.rotation * right;
                
                // Project movement onto surface if grounded
                let final_forward = if self.is_grounded {
                    let projected = rotated_forward - self.last_ground_normal * rotated_forward.dot(&self.last_ground_normal);
                    if projected.magnitude() > 0.1 { projected.normalize() } else { rotated_forward }
                } else {
                    rotated_forward
                };
                
                let final_right = if self.is_grounded {
                    let projected = rotated_right - self.last_ground_normal * rotated_right.dot(&self.last_ground_normal);
                    if projected.magnitude() > 0.1 { projected.normalize() } else { rotated_right }
                } else {
                    rotated_right
                };
                
                // Calculate movement vector
                let movement: Vector3<f32> = final_forward * move_dir.z + final_right * move_dir.x;
                
                // Apply movement force
                if self.is_grounded {
                    let ground_accel = 80.0f32; // Increased for more responsive movement
                    new_velocity += movement * ground_accel * 0.016f32;
                    
                    // Apply friction when not moving
                    if move_dir.magnitude() == 0.0 {
                        new_velocity.x *= 0.85; // Friction
                        new_velocity.z *= 0.85;
                    }
                } else {
                    // Air control
                    let air_control = 1.0f32;
                    new_velocity += movement * air_control * 0.016f32;
                    
                    // Air resistance
                    new_velocity.x *= 0.99;
                    new_velocity.z *= 0.99;
                }
                
                // Set the final velocity
                body.set_linvel(new_velocity, true);
                
                // Log movement for debugging
                tracing::debug!(
                    "Player {} movement applied - new vel: [{:.1}, {:.1}, {:.1}], grounded: {}, input: forward={}, backward={}, left={}, right={}",
                    self.id, new_velocity.x, new_velocity.y, new_velocity.z, self.is_grounded,
                    self.input.forward, self.input.backward, self.input.left, self.input.right
                );
            }
            
            // Handle jump
            if self.input.jump && self.is_grounded {
                let jump_impulse = self.last_ground_normal * 8.0; // Increased jump force
                let current_vel = body.linvel();
                let new_vel = Vector3::new(current_vel.x, current_vel.y, current_vel.z) + jump_impulse;
                body.set_linvel(new_vel, true);
                tracing::debug!("Player {} jumped with impulse: [{:.1}, {:.1}, {:.1}]", 
                    self.id, jump_impulse.x, jump_impulse.y, jump_impulse.z);
            }
            
            // Apply yaw rotation for visual feedback
            if self.is_grounded && self.input.yaw.abs() > 0.001 {
                let yaw_rotation = UnitQuaternion::from_axis_angle(
                    &Unit::new_normalize(self.last_ground_normal),
                    self.input.yaw * 0.1 // Increased rotation speed
                );
                self.rotation = yaw_rotation * self.rotation;
                body.set_rotation(self.rotation, true);
            }
            
            // Check if we need to shift the player's local origin
            let local_pos_vec = self.position - Point3::origin();
            let local_distance = local_pos_vec.magnitude();
            if local_distance > 500.0 {
                let shift = local_pos_vec;
                self.world_origin += shift;
                
                let new_local_pos = Point3::origin();
                body.set_translation(vector![new_local_pos.x, new_local_pos.y, new_local_pos.z], true);
                self.position = new_local_pos;
                
                tracing::info!("Shifted player {} origin by {:?}, new world origin: {:?}", 
                    self.id, shift, self.world_origin);
            }
        }
    }
    
    pub fn remove_from_world(&self, rigid_body_set: &mut RigidBodySet, collider_set: &mut ColliderSet) {
        tracing::info!("Removing player {} from physics world", self.id);
        
        // Remove collider first - check if it exists
        if collider_set.get(self.collider_handle).is_some() {
            tracing::debug!("Removing collider for player {}", self.id);
            collider_set.remove(
                self.collider_handle,
                &mut IslandManager::new(),
                rigid_body_set,
                true,
            );
        } else {
            tracing::warn!("Collider for player {} not found during removal (may already be removed)", self.id);
        }
        
        // Then remove rigid body - check if it exists
        if rigid_body_set.get(self.body_handle).is_some() {
            tracing::debug!("Removing rigid body for player {}", self.id);
            rigid_body_set.remove(
                self.body_handle,
                &mut IslandManager::new(),
                collider_set,
                &mut ImpulseJointSet::new(),
                &mut MultibodyJointSet::new(),
                true,
            );
        } else {
            tracing::warn!("Rigid body for player {} not found during removal (may already be removed)", self.id);
        }
        
        tracing::info!("Successfully removed player {} from physics world", self.id);
    }
}