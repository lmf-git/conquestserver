use nalgebra::{Point3, UnitQuaternion, Vector3, Unit};
use rapier3d::prelude::*;
use uuid::Uuid;

use crate::messages::{PlayerState, PlayerInput};

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
}

impl Player {
    pub fn new(id: Uuid, rigid_body_set: &mut RigidBodySet, collider_set: &mut ColliderSet) -> Self {
        // Randomize spawn position to avoid overlap with other players
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};
        
        let mut hasher = DefaultHasher::new();
        id.hash(&mut hasher);
        let hash = hasher.finish();
        
        // Spread players around a circle on the platform
        let angle = (hash as f32 / u64::MAX as f32) * 2.0 * std::f32::consts::PI;
        let radius = 5.0; // Keep them on the platform (platform is 50x50)
        let spawn_x = angle.cos() * radius;
        let spawn_z = angle.sin() * radius;
        let spawn_y = 40.0; // Higher spawn to ensure they're above platform
        
        let spawn_position = vector![spawn_x, spawn_y, spawn_z];
        
        tracing::info!("Creating player {} at position: [{:.1}, {:.1}, {:.1}]", 
            id, spawn_position.x, spawn_position.y, spawn_position.z);
        
        // Create player rigid body - DYNAMIC for proper gravity application but with locked rotations
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(spawn_position)
            .linear_damping(0.5) // Much higher damping for stability
            .angular_damping(5.0) // Much higher angular damping
            .can_sleep(false)
            .lock_rotations() // Lock rotations to prevent tumbling while allowing gravity forces
            .build();
        
        let body_handle = rigid_body_set.insert(rigid_body);
        
        // Create player capsule collider with more conservative settings
        let player_height = 1.8;
        let player_radius = 0.4;
        
        let collider = ColliderBuilder::capsule_y(
            player_height / 2.0 - player_radius,
            player_radius,
        )
        .friction(0.3) // More friction for stability
        .restitution(0.0)
        .density(0.8) // Lighter for less impact forces
        .active_collision_types(ActiveCollisionTypes::default())
        .active_events(ActiveEvents::COLLISION_EVENTS)
        .build();
        
        let collider_handle = collider_set.insert_with_parent(
            collider,
            body_handle,
            rigid_body_set,
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
        }
    }
    
    pub fn apply_input(&mut self, input: PlayerInput, sequence: u32) {
        self.input_sequence = sequence;
        
        // Extract world position before moving input
        let client_world_pos = Vector3::new(
            input.world_position[0],
            input.world_position[1],
            input.world_position[2]
        );
        
        self.world_origin = Vector3::new(
            input.world_origin[0],
            input.world_origin[1], 
            input.world_origin[2]
        );
        
        // Now we can move input
        self.input = input;
        
        // Convert to local position
        let local_pos = client_world_pos - self.world_origin;
        self.position = Point3::from(local_pos);
    }
    
    pub fn update_physics(&mut self, rigid_body_set: &mut RigidBodySet, collider_set: &ColliderSet) {
        // First, check grounding without holding a mutable borrow
        self.check_grounded(rigid_body_set, collider_set);
        
        // Then update physics with mutable access
        if let Some(body) = rigid_body_set.get_mut(self.body_handle) {
            // Update position and velocity from physics body
            let translation = body.translation();
            let linvel = body.linvel();
            let rotation = body.rotation();
            
            self.position = Point3::new(translation.x, translation.y, translation.z);
            self.velocity = Vector3::new(linvel.x, linvel.y, linvel.z);
            self.rotation = *rotation;
            
            // Apply movement based on input - this was missing for other players!
            let current_vel = body.linvel();
            let mut new_velocity = Vector3::new(current_vel.x, current_vel.y, current_vel.z);
            
            // Apply movement based on input
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
                let ground_accel = 100.0f32;
                new_velocity += movement * ground_accel * 0.016f32;
                
                // Apply friction when not moving
                if move_dir.magnitude() == 0.0 {
                    new_velocity.x *= 0.8;
                    new_velocity.z *= 0.8;
                }
                
                // Clamp velocity against gravity if grounded
                let world_pos = self.position + self.world_origin;
                let planet_center = Point3::new(0.0, -250.0, 0.0);
                let to_planet = planet_center - world_pos;
                let gravity_dir = to_planet.normalize();
                
                let downward_vel = new_velocity.dot(&gravity_dir);
                if downward_vel > 5.0 { // Prevent excessive downward velocity when grounded
                    new_velocity -= gravity_dir * (downward_vel - 2.0);
                }
            } else {
                // Air control
                let air_control = 1.0f32;
                new_velocity += movement * air_control * 0.016f32;
                
                // Air resistance
                new_velocity.x *= 0.95;
                new_velocity.z *= 0.95;
            }
            
            // Handle jump
            if self.input.jump && self.is_grounded {
                let jump_impulse = self.last_ground_normal * 8.0;
                new_velocity += jump_impulse;
                tracing::debug!("Player {} jumped with impulse: [{:.1}, {:.1}, {:.1}]", 
                    self.id, jump_impulse.x, jump_impulse.y, jump_impulse.z);
            }
            
            // Apply yaw rotation for visual feedback
            if self.is_grounded && self.input.yaw.abs() > 0.001 {
                let yaw_rotation = UnitQuaternion::from_axis_angle(
                    &Unit::new_normalize(self.last_ground_normal),
                    self.input.yaw * 0.1 // Reduce rotation speed
                );
                self.rotation = yaw_rotation * self.rotation;
                body.set_rotation(self.rotation, true);
            }
            
            // Set the final velocity
            body.set_linvel(new_velocity, true);
            
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
    
    fn check_grounded(&mut self, rigid_body_set: &RigidBodySet, collider_set: &ColliderSet) {
        // Use planet-centered gravity for ground detection (same as physics world)
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
            let max_distance = 1.5; // Increased raycast distance
            
            // Convert back to local space for raycasting
            let local_ray_origin = Point3::from(local_pos.coords);
            let ray = Ray::new(local_ray_origin, ray_dir);
            let filter = QueryFilter::default().exclude_collider(self.collider_handle);
            
            // Create a temporary query pipeline for raycasting
            let mut query_pipeline = QueryPipeline::new();
            query_pipeline.update(rigid_body_set, collider_set);
            
            // Also check velocity - if moving slowly in gravity direction, likely grounded
            let velocity = body.linvel();
            let velocity_in_gravity_dir = velocity.dot(&ray_dir);
            let slow_descent = velocity_in_gravity_dir < 3.0;
            
            if let Some((_handle, toi)) = query_pipeline.cast_ray(
                rigid_body_set,
                collider_set,
                &ray,
                max_distance,
                true,
                filter,
            ) {
                // Ground hit found
                self.is_grounded = toi < 1.0 && slow_descent;
                
                // Store the gravity direction as the ground normal (inverted)
                self.last_ground_normal = -ray_dir;
                
                if self.is_grounded {
                    tracing::debug!(
                        "Player {} grounded - Distance to ground: {:.2}, Velocity in gravity dir: {:.2}",
                        self.id, toi, velocity_in_gravity_dir
                    );
                }
            } else {
                self.is_grounded = false;
            }
        }
    }
    
    pub fn get_state(&self) -> PlayerState {
        // Convert local position to world position for network transmission
        let world_position = self.position + self.world_origin;
        
        PlayerState {
            position: [world_position.x, world_position.y, world_position.z],
            rotation: [
                self.rotation.i,
                self.rotation.j,
                self.rotation.k,
                self.rotation.w,
            ],
            velocity: [self.velocity.x, self.velocity.y, self.velocity.z],
            is_grounded: self.is_grounded,
            input_sequence: self.input_sequence,
            world_origin: [self.world_origin.x, self.world_origin.y, self.world_origin.z],
        }
    }
    
    pub fn remove_from_world(&self, rigid_body_set: &mut RigidBodySet, collider_set: &mut ColliderSet) {
        // Remove collider first
        if let Some(_collider) = collider_set.get(self.collider_handle) {
            collider_set.remove(
                self.collider_handle,
                &mut IslandManager::new(),
                rigid_body_set,
                true,
            );
        }
        
        // Then remove rigid body
        if let Some(_body) = rigid_body_set.get(self.body_handle) {
            rigid_body_set.remove(
                self.body_handle,
                &mut IslandManager::new(),
                collider_set,
                &mut ImpulseJointSet::new(),
                &mut MultibodyJointSet::new(),
                true,
            );
        }
    }
}
