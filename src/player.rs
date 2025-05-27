use nalgebra::{Point3, UnitQuaternion, Vector3};
use rapier3d::prelude::*;
use uuid::Uuid;
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use crate::messages::PlayerInput;

#[derive(Debug, Clone)]
pub struct Player {
    #[allow(dead_code)]
    pub id: Uuid,
    pub body_handle: RigidBodyHandle,
    pub collider_handle: ColliderHandle,
    pub input_sequence: u32,
    pub world_origin: Vector3<f32>,
    pub is_grounded: bool,
    pub position: Point3<f32>,
    pub velocity: Vector3<f32>,
    pub rotation: UnitQuaternion<f32>,
    pub input: PlayerInput,
    #[allow(dead_code)] // May be used in future for surface alignment
    pub last_ground_normal: Vector3<f32>,
    #[allow(dead_code)] // May be used in future for timing-based grounding
    pub last_ground_contact: f32,
    #[allow(dead_code)] // May be used in future for dynamic height adjustments
    pub player_height: f32,
}

impl Player {
    pub fn new(id: Uuid, rigid_body_set: &mut RigidBodySet, collider_set: &mut ColliderSet) -> Self {
        let player_height = 1.8;
        let player_radius = 0.4;
        
        // Generate deterministic spawn position based on player ID
        let mut hasher = DefaultHasher::new();
        id.hash(&mut hasher);
        let hash = hasher.finish();
        
        // Spread players around a circle on the platform with MORE spacing
        let angle = (hash as f32 / u64::MAX as f32) * 2.0 * std::f32::consts::PI;
        let radius = 8.0; // Increased radius to avoid overlaps
        let spawn_x = angle.cos() * radius;
        let spawn_z = angle.sin() * radius;
        let spawn_y = 35.0; // Platform height + some margin
        
        // Create dynamic rigid body for the player
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![spawn_x, spawn_y, spawn_z])
            .linear_damping(0.1)
            .angular_damping(1.0)
            .can_sleep(false)
            .lock_rotations() // Prevent tumbling
            .build();
        
        let body_handle = rigid_body_set.insert(rigid_body);
        
        // Create capsule collider for the player - use capsule_y for vertical capsule
        let collider = ColliderBuilder::capsule_y(
            player_height / 2.0 - player_radius,
            player_radius
        )
        .friction(0.0)
        .restitution(0.0)
        .density(1.0)
        .active_collision_types(ActiveCollisionTypes::all()) // Use all() instead of DEFAULT
        .active_events(ActiveEvents::COLLISION_EVENTS)
        .build();
        
        let collider_handle = collider_set.insert_with_parent(collider, body_handle, rigid_body_set);
        
        // Initialize with default input
        let default_input = PlayerInput {
            forward: false,
            backward: false,
            left: false,
            right: false,
            jump: false,
            run: false,
            yaw: 0.0,
            pitch: 0.0,
            world_position: [spawn_x, spawn_y, spawn_z],
            world_origin: [0.0, 0.0, 0.0],
        };
        
        Self {
            id,
            body_handle,
            collider_handle,
            input_sequence: 0,
            world_origin: Vector3::zeros(),
            is_grounded: false,
            position: Point3::new(spawn_x, spawn_y, spawn_z),
            velocity: Vector3::zeros(),
            rotation: UnitQuaternion::identity(),
            input: default_input,
            last_ground_normal: Vector3::new(0.0, 1.0, 0.0),
            last_ground_contact: 0.0,
            player_height,
        }
    }
    
    fn check_grounded(&mut self, rigid_body_set: &RigidBodySet, _collider_set: &ColliderSet) {
        if let Some(body) = rigid_body_set.get(self.body_handle) {
            let velocity = *body.linvel();
            let position = *body.translation();
            
            // Use planet-centered gravity direction for ground check (same as client)
            let planet_center = Point3::new(0.0, -250.0, 0.0);
            let to_planet = planet_center - Point3::new(position.x, position.y, position.z);
            let gravity_dir = to_planet.normalize();
            
            // Multiple foot position raycasts (like client)
            let player_quat = *body.rotation();
            let foot_offset = self.player_height * 0.2; // Use player_height from struct
            let foot_level = -self.player_height * 0.5;
            
            // Calculate foot positions in world space
            let forward = Vector3::new(0.0, 0.0, -1.0);
            let right = Vector3::new(1.0, 0.0, 0.0);
            
            // Apply player rotation to get local directions
            let _player_forward = player_quat * forward;
            let player_right = player_quat * right;
            let player_up = player_quat * Vector3::new(0.0, 1.0, 0.0);
            
            let player_center = Point3::new(position.x, position.y, position.z);
            let foot_base = player_center + player_up * foot_level;
            
            let left_foot_pos = foot_base - player_right * foot_offset;
            let right_foot_pos = foot_base + player_right * foot_offset;
            let center_foot_pos = foot_base;
            
            // Cast rays like client (use query pipeline for raycasting)
            let ray_distance = 1.5; // Match client raycast distance
            
            let left_ray = Ray::new(left_foot_pos.into(), gravity_dir.into());
            let right_ray = Ray::new(right_foot_pos.into(), gravity_dir.into());
            let center_ray = Ray::new(center_foot_pos.into(), gravity_dir.into());
            
            // Perform raycasts (excluding self)
            let left_hit = self.cast_ray_excluding_self(&left_ray, ray_distance, _collider_set);
            let right_hit = self.cast_ray_excluding_self(&right_ray, ray_distance, _collider_set);
            let center_hit = self.cast_ray_excluding_self(&center_ray, ray_distance, _collider_set);
            
            // Check conditions like client
            let has_ray_hits = left_hit || right_hit || center_hit;
            let low_downward_velocity = velocity.dot(&gravity_dir) < 3.0; // Match client threshold
            let recent_ground_contact = (self.last_ground_contact - 0.0).abs() < 0.2; // Simple time check
            
            // Use same grounding logic as client - conservative approach
            self.is_grounded = (has_ray_hits && low_downward_velocity) ||
                              (recent_ground_contact && velocity.dot(&gravity_dir).abs() < 0.5);
            
            // Update last ground contact if we have hits
            if has_ray_hits {
                self.last_ground_contact = 1.0; // Simple flag for now
            } else {
                self.last_ground_contact = 0.0;
            }
        } else {
            // Fallback if no body
            self.is_grounded = false;
        }
    }
    
    // Helper function for raycasting excluding self
    fn cast_ray_excluding_self(&self, ray: &Ray, max_distance: f32, _collider_set: &ColliderSet) -> bool {
        // Simple implementation - in a real scenario you'd use the query pipeline
        // For now, just check if we're close to the platform or planet
        let ray_end = ray.origin + ray.dir * max_distance;
        
        // Check if ray hits platform level (y=30)
        if ray.origin.y > 30.0 && ray_end.y <= 31.5 {
            // Ray crosses platform level
            return true;
        }
        
        // Check if ray hits planet surface (rough approximation)
        let planet_center = Point3::new(0.0, -250.0, 0.0);
        let to_planet = planet_center - ray.origin;
        let distance_to_planet = to_planet.magnitude();
        
        if distance_to_planet < 220.0 { // Rough planet surface check
            return true;
        }
        
        false
    }

    pub fn apply_input(&mut self, input: PlayerInput, sequence: u32) {
        self.input = input;
        self.input_sequence = sequence;
    }

    pub fn update_physics(&mut self, rigid_body_set: &mut RigidBodySet, collider_set: &ColliderSet) {
        // Get position and velocity data first to avoid borrowing conflicts
        let (pos, vel, rot) = if let Some(body) = rigid_body_set.get(self.body_handle) {
            (*body.translation(), *body.linvel(), *body.rotation())
        } else {
            return; // Exit early if no body
        };
        
        // Check for physics instability - teleport back if position is extreme
        if !pos.x.is_finite() || !pos.y.is_finite() || !pos.z.is_finite() ||
           pos.magnitude() > 10000.0 {
            tracing::warn!("Player {} position is unstable: [{:.1}, {:.1}, {:.1}], teleporting back", self.id, pos.x, pos.y, pos.z);
            if let Some(body) = rigid_body_set.get_mut(self.body_handle) {
                body.set_translation(vector![0.0, 35.0, 0.0], true);
                body.set_linvel(vector![0.0, 0.0, 0.0], true);
            }
            return;
        }
        
        // Check for velocity instability
        if !vel.x.is_finite() || !vel.y.is_finite() || !vel.z.is_finite() ||
           vel.magnitude() > 100.0 {
            tracing::warn!("Player {} velocity is unstable: [{:.1}, {:.1}, {:.1}], clamping", self.id, vel.x, vel.y, vel.z);
            if let Some(body) = rigid_body_set.get_mut(self.body_handle) {
                let clamped_vel = if vel.magnitude() > 0.1 {
                    vel.normalize() * 10.0 // Clamp to reasonable speed
                } else {
                    vector![0.0, 0.0, 0.0]
                };
                body.set_linvel(clamped_vel, true);
            }
            return;
        }
        
        // Update stored state
        self.position = Point3::new(pos.x, pos.y, pos.z);
        self.velocity = Vector3::new(vel.x, vel.y, vel.z);
        self.rotation = rot;
        
        // Check grounded state using the sophisticated client-like system
        self.check_grounded(rigid_body_set, collider_set);
        
        // Now get mutable access to apply forces
        if let Some(body) = rigid_body_set.get_mut(self.body_handle) {
            // Apply movement based on input with much more conservative forces
            if self.input.forward || self.input.backward || self.input.left || self.input.right {
                let player_quat = UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
                    rot.w, rot.i, rot.j, rot.k
                ));
                
                let mut move_dir = Vector3::zeros();
                
                if self.input.forward {
                    move_dir += player_quat * Vector3::new(0.0, 0.0, -1.0);
                }
                if self.input.backward {
                    move_dir += player_quat * Vector3::new(0.0, 0.0, 1.0);
                }
                if self.input.left {
                    move_dir += player_quat * Vector3::new(-1.0, 0.0, 0.0);
                }
                if self.input.right {
                    move_dir += player_quat * Vector3::new(1.0, 0.0, 0.0);
                }
                
                if move_dir.magnitude() > 0.0 {
                    move_dir = move_dir.normalize();
                    let speed = if self.input.run { 12.0 } else { 6.0 }; // Reduced speeds
                    
                    if self.is_grounded {
                        // Much more conservative ground movement
                        let force = move_dir * speed * 20.0; // Reduced force
                        body.add_force(vector![force.x, force.y, force.z], true);
                    } else {
                        // Very limited air control
                        let force = move_dir * speed * 2.0; // Very reduced air control
                        body.add_force(vector![force.x, force.y, force.z], true);
                    }
                }
            }
            
            // Handle jumping with reduced force
            if self.input.jump && self.is_grounded {
                let planet_center = Point3::new(0.0, -250.0, 0.0);
                let to_planet = planet_center - Point3::new(pos.x, pos.y, pos.z);
                let gravity_dir = to_planet.normalize();
                let jump_dir = -gravity_dir; // Opposite to gravity
                
                let jump_force = jump_dir * 5.0; // Reduced jump force
                body.add_force(vector![jump_force.x, jump_force.y, jump_force.z], true);
            }
        }
    }
    
    pub fn remove_from_world(&self, rigid_body_set: &mut RigidBodySet, collider_set: &mut ColliderSet) {
        // IMPORTANT: Check if bodies still exist before trying to remove them
        
        // Remove collider first - but only if it exists
        if collider_set.get(self.collider_handle).is_some() {
            let mut island_manager = IslandManager::new();
            collider_set.remove(
                self.collider_handle, 
                &mut island_manager, 
                rigid_body_set, 
                true
            );
            tracing::debug!("Removed collider for player {}", self.id);
        } else {
            tracing::warn!("Collider {} for player {} was already removed", self.collider_handle.into_raw_parts().0, self.id);
        }
        
        // Remove rigid body - but only if it exists
        if rigid_body_set.get(self.body_handle).is_some() {
            let mut island_manager = IslandManager::new();
            let mut impulse_joint_set = ImpulseJointSet::new();
            let mut multibody_joint_set = MultibodyJointSet::new();
            
            rigid_body_set.remove(
                self.body_handle,
                &mut island_manager,
                collider_set,
                &mut impulse_joint_set,
                &mut multibody_joint_set,
                true
            );
            tracing::debug!("Removed rigid body for player {}", self.id);
        } else {
            tracing::warn!("Rigid body {} for player {} was already removed", self.body_handle.into_raw_parts().0, self.id);
        }
    }
}