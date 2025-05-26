use nalgebra::{Point3, UnitQuaternion, Vector3, Unit};
use rapier3d::prelude::*;
use uuid::Uuid;

use crate::messages::{PlayerState, PlayerInput};

#[derive(Clone)]
pub struct Player {
    #[allow(dead_code)]  // We use this for logging and identification
    pub id: Uuid,
    pub body_handle: RigidBodyHandle,
    pub collider_handle: ColliderHandle,
    pub position: Point3<f32>,
    pub rotation: UnitQuaternion<f32>,
    pub velocity: Vector3<f32>,
    pub input: PlayerInput,
    pub input_sequence: u32,
    pub is_grounded: bool,
    pub last_ground_normal: Vector3<f32>,
    pub world_origin: Vector3<f32>,
}

impl Player {
    pub fn new(id: Uuid, rigid_body_set: &mut RigidBodySet, collider_set: &mut ColliderSet) -> Self {
        // Player spawn position (above platform)
        let spawn_position = vector![0.0, 35.0, 0.0];
        
        tracing::info!("Creating player with ID: {}", id); // Use the id field
        
        // Create player rigid body
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(spawn_position)
            .linear_damping(0.1)
            .angular_damping(1.0)
            .locked_axes(LockedAxes::ROTATION_LOCKED)
            .can_sleep(false)
            .build();
        
        let body_handle = rigid_body_set.insert(rigid_body);
        
        // Create player capsule collider
        let player_height = 1.8;
        let player_radius = 0.4;
        
        let collider = ColliderBuilder::capsule_y(
            player_height / 2.0 - player_radius,
            player_radius,
        )
        .friction(0.0)
        .restitution(0.0)
        .density(1.0)
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
            position: Point3::from(spawn_position),
            rotation: UnitQuaternion::identity(),
            velocity: Vector3::zeros(),
            input: PlayerInput {
                forward: false,
                backward: false,
                left: false,
                right: false,
                jump: false,
                run: false,
                yaw: 0.0,
                pitch: 0.0,
                world_position: [0.0, 0.0, 0.0],
                world_origin: [0.0, 0.0, 0.0],
            },
            input_sequence: 0,
            is_grounded: false,
            last_ground_normal: Vector3::y(),
            world_origin: Vector3::zeros(), // Initialize origin offset
        }
    }
    
    pub fn apply_input(&mut self, input: PlayerInput, sequence: u32) {
        // Update world origin if provided
        self.world_origin = Vector3::new(input.world_origin[0], input.world_origin[1], input.world_origin[2]);
        
        self.input = input;
        self.input_sequence = sequence;
    }
    
    pub fn update_physics(&mut self, rigid_body_set: &mut RigidBodySet, collider_set: &ColliderSet) {
        // First, check grounding without holding a mutable borrow
        self.check_grounded(rigid_body_set, collider_set);
        
        // Then update physics with mutable access
        if let Some(body) = rigid_body_set.get_mut(self.body_handle) {
            // Update position and velocity from physics body
            let translation = body.translation();
            self.position = Point3::new(translation.x, translation.y, translation.z);
            self.velocity = *body.linvel();
            
            // Check if we need to shift the player's local origin
            let local_distance = self.position.coords.magnitude();
            if local_distance > 500.0 { // Half of client's ORIGIN_SHIFT_THRESHOLD
                // Shift the origin
                let shift = self.position.coords;
                self.world_origin += shift;
                
                // Reset local position to near origin
                let new_local_pos = Point3::origin();
                body.set_translation(new_local_pos.coords, true);
                self.position = new_local_pos;
                
                tracing::info!("Shifted player {} origin by {:?}, new world origin: {:?}", 
                    self.id, shift, self.world_origin);
            }
            
            // Apply planet-centered gravity (matching client implementation)
            let planet_center = Point3::new(0.0, -250.0, 0.0);
            let gravity_strength = 25.0;
            let to_planet = planet_center - self.position;
            let gravity_dir = to_planet.normalize();
            let gravity_force = gravity_dir * gravity_strength * 0.016f32; // 60Hz frame time
            
            // Apply gravity to velocity
            let current_vel = body.linvel();
            let new_velocity = Vector3::new(
                current_vel.x + gravity_force.x,
                current_vel.y + gravity_force.y,
                current_vel.z + gravity_force.z,
            );
            
            // Apply movement based on input - fix type annotation
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
            
            // Calculate movement vector with explicit type
            let movement: Vector3<f32> = final_forward * move_dir.z + final_right * move_dir.x;
            
            // Apply movement force
            let mut final_velocity = new_velocity;
            
            if self.is_grounded {
                let ground_accel = 100.0f32;
                final_velocity += movement * ground_accel * 0.016f32;
                
                // Apply friction when not moving
                if move_dir.magnitude() == 0.0 {
                    final_velocity.x *= 0.8;
                    final_velocity.y *= 0.95;
                    final_velocity.z *= 0.8;
                }
            } else {
                // Air control
                let air_control = 1.0f32;
                final_velocity += movement * air_control * 0.016f32;
                
                // Air resistance
                final_velocity.x *= 0.95;
                final_velocity.y *= 0.98;
                final_velocity.z *= 0.95;
            }
            
            // Handle jump
            if self.input.jump && self.is_grounded {
                let jump_impulse = self.last_ground_normal * 8.0;
                final_velocity += jump_impulse;
            }
            
            // Set the final velocity
            body.set_linvel(final_velocity, true);
            
            // Update rotation based on mouse input (only yaw when grounded)
            if self.is_grounded {
                // Create unit vector from ground normal for axis-angle rotation
                let axis = if self.last_ground_normal.magnitude() > 0.1 {
                    Unit::new_normalize(self.last_ground_normal)
                } else {
                    Unit::new_normalize(Vector3::y()) // Default up axis
                };
                
                let yaw_rotation = UnitQuaternion::from_axis_angle(&axis, self.input.yaw);
                self.rotation = yaw_rotation * self.rotation;
                body.set_rotation(self.rotation, true);
            }
        }
    }
    
    fn check_grounded(&mut self, rigid_body_set: &RigidBodySet, collider_set: &ColliderSet) {
        // Simple ground check using raycast
        let ray_origin = self.position;
        let ray_dir = -Vector3::y();
        let max_distance = 1.0;
        
        let ray = Ray::new(ray_origin, ray_dir);
        let filter = QueryFilter::default().exclude_collider(self.collider_handle);
        
        // Create a temporary query pipeline for raycasting
        let mut query_pipeline = QueryPipeline::new();
        query_pipeline.update(rigid_body_set, collider_set);
        
        if let Some((_handle, _toi)) = query_pipeline.cast_ray(
            rigid_body_set,
            collider_set,
            &ray,
            max_distance,
            true,
            filter,
        ) {
            self.is_grounded = true;
        } else {
            self.is_grounded = false;
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
    
    // Add method to get local position (for physics calculations)
    pub fn get_local_position(&self) -> Point3<f32> {
        self.position
    }
    
    // Add method to convert world position to local position
    pub fn world_to_local(&self, world_pos: Point3<f32>) -> Point3<f32> {
        world_pos - self.world_origin
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
