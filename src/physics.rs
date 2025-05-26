use nalgebra::{Point3, Vector3};
use rapier3d::prelude::*;
use uuid::Uuid;
use std::collections::HashMap;

use crate::player::Player;

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
        };
        
        // Create game world geometry
        physics.create_planet();
        physics.create_platform();
        
        physics
    }
    
    pub fn add_player(&mut self, id: Uuid) -> Player {
        let player = Player::new(id, &mut self.rigid_body_set, &mut self.collider_set);
        self.players.insert(id, player.clone());
        player
    }
    
    pub fn remove_player(&mut self, id: Uuid) {
        if let Some(player) = self.players.remove(&id) {
            player.remove_from_world(&mut self.rigid_body_set, &mut self.collider_set);
        }
    }
    
    pub fn step(&mut self) {
        // Apply planet-centered gravity to all dynamic bodies
        let planet_center = Point3::new(0.0, -250.0, 0.0);
        let gravity_strength = 25.0;
        
        // First, collect handles and world positions for dynamic bodies
        let mut body_data: Vec<(RigidBodyHandle, Vector3<f32>)> = Vec::new();
        
        for (handle, body) in self.rigid_body_set.iter() {
            if body.body_type() == RigidBodyType::Dynamic {
                let position = body.translation();
                
                // Check if this is a player body and get their world origin
                let world_origin = self.players.values()
                    .find(|p| p.body_handle == handle)
                    .map(|p| p.world_origin)
                    .unwrap_or(Vector3::zeros());
                
                // Calculate world position
                let world_position = position + world_origin;
                body_data.push((handle, world_position));
            }
        }
        
        // Now apply gravity to each dynamic body
        for (handle, world_position) in body_data {
            if let Some(body) = self.rigid_body_set.get_mut(handle) {
                let to_planet = planet_center - Point3::from(world_position);
                let gravity_dir = to_planet.normalize();
                let gravity_force = gravity_dir * gravity_strength * body.mass();
                
                body.add_force(gravity_force, true);
            }
        }
        
        // Step the physics simulation
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
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
        
        // Update player physics
        let player_ids: Vec<Uuid> = self.players.keys().cloned().collect();
        for player_id in player_ids {
            if let Some(player) = self.players.get_mut(&player_id) {
                player.update_physics(&mut self.rigid_body_set, &self.collider_set);
            }
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
        // Main platform
        let platform_half_extents = vector![25.0, 1.5, 25.0];
        
        let rigid_body = RigidBodyBuilder::fixed()
            .translation(vector![0.0, 30.0, 0.0])
            .build();
        
        let handle = self.rigid_body_set.insert(rigid_body);
        
        let collider = ColliderBuilder::cuboid(
            platform_half_extents.x,
            platform_half_extents.y,
            platform_half_extents.z,
        )
        .friction(0.8)
        .restitution(0.2)
        .build();
        
        self.collider_set.insert_with_parent(collider, handle, &mut self.rigid_body_set);
    }
}
