use nalgebra::{Point3, UnitQuaternion, Vector3};
use rapier3d::prelude::*;
use uuid::Uuid;

use crate::messages::PlayerInput;

#[derive(Debug, Clone)]
pub struct Player {
    #[allow(dead_code)]  // Used for identification but not directly read
    pub id: Uuid,
    pub body_handle: RigidBodyHandle,
    pub collider_handle: ColliderHandle,
    pub position: Point3<f32>,
    pub velocity: Vector3<f32>,
    pub rotation: UnitQuaternion<f32>,
    pub input: PlayerInput,
    pub input_sequence: u32,
    pub is_grounded: bool,
    pub world_origin: Vector3<f32>,
}

impl Player {
    pub fn new(id: Uuid, rigid_body_set: &mut RigidBodySet, collider_set: &mut ColliderSet) -> Self {
        // Create player physics body at spawn position
        let spawn_position = vector![0.0, 35.0, 0.0]; // High spawn to fall onto platform
        
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(spawn_position)
            .lock_rotations()
            .linear_damping(0.1)
            .angular_damping(1.0)
            .can_sleep(false)
            .build();
        
        let body_handle = rigid_body_set.insert(rigid_body);
        
        let collider = ColliderBuilder::capsule_y(0.4, 0.4)
            .friction(0.0)
            .restitution(0.0)
            .density(1.0)
            .build();
        
        let collider_handle = collider_set.insert_with_parent(collider, body_handle, rigid_body_set);
        
        let player = Player {
            id,
            body_handle,
            collider_handle,
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
            input_sequence: 0,
            is_grounded: false,
            world_origin: Vector3::zeros(),
        };
        
        tracing::info!("Created player {} at position: [{:.1}, {:.1}, {:.1}]", 
                      id, spawn_position.x, spawn_position.y, spawn_position.z);
        
        player
    }
    
    pub fn apply_input(&mut self, input: PlayerInput, sequence: u32) {
        self.input = input;
        self.input_sequence = sequence;
    }
    
    pub fn update_physics(&mut self, body: &RigidBody) {
        let translation = body.translation();
        let velocity = body.linvel();
        let rotation = body.rotation();
        
        self.position = Point3::new(translation.x, translation.y, translation.z);
        self.velocity = Vector3::new(velocity.x, velocity.y, velocity.z);
        self.rotation = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            rotation.w, rotation.i, rotation.j, rotation.k
        ));
    }
}