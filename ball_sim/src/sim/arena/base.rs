use glam::Vec3A;

use super::ArenaContactTracker;
use crate::{
    ARENA_COLLISION_SHAPES, ArenaConfig, ArenaMemWeightMode, BallHitWorldEvent, GameMode,
    MutatorConfig, Team,
    bullet::{
        collision::{
            broadphase::{GridBroadphase, HashedOverlappingPairCache},
            dispatch::collision_dispatcher::CollisionDispatcher,
            shapes::{collision_shape::CollisionShapes, static_plane_shape::StaticPlaneShape},
        },
        dynamics::{
            constraint_solver::seq_impulse_constraint_solver::SeqImpulseConstraintSolver,
            discrete_dynamics_world::DiscreteDynamicsWorld,
            rigid_body::{RigidBody, RigidBodyConstructionInfo},
        },
    },
    consts::{self, BT_TO_UU, TICK_TIME, UU_TO_BT},
    sim::{Ball, BallState, arena::ArenaEventList},
};

#[derive(Clone)]
pub struct Arena {
    bullet_world: DiscreteDynamicsWorld,
    ball: Ball,
    tick_count: u64,
    game_mode: GameMode,
    mutator_config: MutatorConfig,
    contact_tracker: ArenaContactTracker,
    events: ArenaEventList,
}

impl Arena {
    pub fn new(game_mode: GameMode) -> Self {
        Self::new_with_config(game_mode, ArenaConfig::DEFAULT)
    }

    pub fn new_with_config(game_mode: GameMode, config: ArenaConfig) -> Self {
        let mutator_config = MutatorConfig::new(game_mode);

        let collision_dispatcher = CollisionDispatcher::default();
        let constraint_solver = SeqImpulseConstraintSolver::default();
        let overlapping_pair_cache = HashedOverlappingPairCache::default();

        let cell_size_multiplier = match config.mem_weight_mode {
            ArenaMemWeightMode::Light => 4.0,
            ArenaMemWeightMode::Heavy => 1.0,
        };

        let broadphase = GridBroadphase::new(
            config.min_pos * UU_TO_BT,
            config.max_pos * UU_TO_BT,
            config.max_aabb_len * UU_TO_BT * cell_size_multiplier,
            overlapping_pair_cache,
        );

        let (ball, body) = Ball::new(game_mode, &mutator_config);
        let mut bullet_world =
            DiscreteDynamicsWorld::new(collision_dispatcher, broadphase, constraint_solver, body);
        bullet_world.set_gravity(mutator_config.gravity * UU_TO_BT);

        if game_mode != GameMode::TheVoid {
            Self::setup_arena_collision_shapes(&mut bullet_world, game_mode);
        }

        Self {
            ball,
            game_mode,
            mutator_config,
            tick_count: 0,
            bullet_world,
            contact_tracker: ArenaContactTracker::default(),
            events: ArenaEventList::default(),
        }
    }

    fn add_static_collision_shape(
        bullet_world: &mut DiscreteDynamicsWorld,
        shape: CollisionShapes,
        pos_bt: Vec3A,
    ) {
        let mut rb_info = RigidBodyConstructionInfo::new(shape);
        rb_info.restitution = consts::arena::BASE_COEFS.restitution;
        rb_info.friction = consts::arena::BASE_COEFS.friction;
        rb_info.start_world_trans = pos_bt;

        let shape_rb = RigidBody::new(rb_info);
        bullet_world.add_rigid_body(shape_rb);
    }

    fn setup_arena_collision_shapes(bullet_world: &mut DiscreteDynamicsWorld, game_mode: GameMode) {
        debug_assert!(game_mode != GameMode::TheVoid);

        let mesh_game_mode = match game_mode {
            GameMode::Heatseeker | GameMode::Snowday => GameMode::Soccar,
            _ => game_mode,
        };
        let collision_shapes = ARENA_COLLISION_SHAPES.read().unwrap();
        let collision_meshes = &collision_shapes
            .as_ref()
            .expect("Arena collision shapes are uninitialized - please call init(..) first.")
            [&mesh_game_mode];
        assert!(
            !collision_meshes.is_empty(),
            "No arena meshes found for the game mode {game_mode:?}"
        );

        for mesh in collision_meshes {
            Self::add_static_collision_shape(
                bullet_world,
                CollisionShapes::TriangleMesh(mesh.clone()),
                Vec3A::ZERO,
            );
        }

        drop(collision_shapes);

        let arena_aabb = consts::arena::get_aabb(game_mode);

        let mut add_plane = |pos_uu: Vec3A, normal: Vec3A| {
            debug_assert!(normal.is_normalized());
            let pos_bt = pos_uu * UU_TO_BT;
            let plane_shape = StaticPlaneShape::new(pos_bt, normal);

            Self::add_static_collision_shape(
                bullet_world,
                CollisionShapes::StaticPlane(plane_shape),
                pos_bt,
            );
        };

        // Ceiling
        add_plane(Vec3A::new(0.0, 0.0, arena_aabb.max.z), Vec3A::NEG_Z);

        if game_mode != GameMode::Dropshot {
            // Floor
            add_plane(Vec3A::new(0.0, 0.0, arena_aabb.min.z), Vec3A::Z);

            // Side walls
            add_plane(
                Vec3A::new(arena_aabb.min.x, 0.0, arena_aabb.center().z),
                Vec3A::X,
            );
            add_plane(
                Vec3A::new(arena_aabb.max.x, 0.0, arena_aabb.center().z),
                Vec3A::NEG_X,
            );
        }

        match game_mode {
            GameMode::Hoops => {
                // Y walls
                add_plane(
                    Vec3A::new(0.0, arena_aabb.min.y, arena_aabb.center().z),
                    Vec3A::Y,
                );

                add_plane(
                    Vec3A::new(0.0, arena_aabb.min.z, arena_aabb.center().z),
                    Vec3A::NEG_Y,
                );
            }
            GameMode::Dropshot => {
                // Add tiles
                todo!()
            }
            _ => {}
        }
    }

    fn ball_within_hoops_goal_xy_margin_eq(x: f32, y: f32) -> f32 {
        const SCALE_Y: f32 = 0.9;
        const OFFSET_Y: f32 = 2770.0;
        const RADIUS_SQ: f32 = 716.0 * 716.0;

        let dy = y.abs() * SCALE_Y - OFFSET_Y;
        let dist_sq = x * x + dy * dy;
        dist_sq - RADIUS_SQ
    }

    pub fn is_ball_scored(&self) -> bool {
        let ball_pos = self.bullet_world.ball().get_world_trans() * BT_TO_UU;

        match self.game_mode {
            GameMode::Soccar | GameMode::Heatseeker | GameMode::Snowday => {
                ball_pos.y.abs()
                    > self.mutator_config.goal_base_threshold_y + self.mutator_config.ball_radius
            }
            GameMode::Hoops => {
                if ball_pos.z < consts::goal::HOOPS_GOAL_SCORE_THRESHOLD_Z {
                    Self::ball_within_hoops_goal_xy_margin_eq(ball_pos.x, ball_pos.y) < 0.0
                } else {
                    false
                }
            }
            GameMode::Dropshot => ball_pos.z < -self.mutator_config.ball_radius * 1.75,
            GameMode::TheVoid => false,
        }
    }

    pub fn reset_to_kickoff(&mut self, team: Team) {
        let mut ball_state = BallState::DEFAULT;
        match self.game_mode {
            GameMode::Heatseeker => {
                let y_sign = f32::from(team as i8 * 2 - 1);
                let scale = Vec3A::new(1.0, y_sign, 1.0);
                ball_state.phys.pos = consts::heatseeker::BALL_START_POS * scale;
                ball_state.phys.vel = consts::heatseeker::BALL_START_VEL * scale;
            }
            GameMode::Snowday => {
                ball_state.phys.vel.z = f32::EPSILON;
            }
            _ => {}
        }

        self.set_ball_state(ball_state);
    }

    /// Steps the arena for 1 tick, returning the events produced during that tick
    pub fn step_tick(&mut self) -> &[BallHitWorldEvent] {
        self.events.clear();

        let ball_rb = &self.bullet_world.ball_mut();
        let should_sleep =
            ball_rb.lin_vel.length_squared() == 0.0 && ball_rb.ang_vel.length_squared() == 0.0;
        if should_sleep {
            self.tick_count += 1;
            return self.get_last_step_events();
        }

        self.ball
            .pre_tick_update(self.bullet_world.ball_mut(), self.game_mode);

        self.bullet_world
            .step_simulation(TICK_TIME, &mut self.contact_tracker);

        let contact_count = self.contact_tracker.num_records();
        for idx in 0..contact_count {
            let manifold_point = &self.contact_tracker.get_record(idx).manifold_point;
            let contact_point = manifold_point.pos_world_on_b * BT_TO_UU;
            let contact_normal = manifold_point.normal_world_on_b;

            self.ball.on_world_hit(contact_normal, self.game_mode);

            self.events.push(BallHitWorldEvent {
                contact_point,
                contact_normal,
            })
        }

        self.contact_tracker.clear_records();

        self.ball
            .finish_physics_tick(self.bullet_world.ball_mut(), &self.mutator_config);

        if self.game_mode == GameMode::Dropshot {
            todo!("Dropshot tile state sync")
        }

        self.tick_count += 1;

        self.get_last_step_events()
    }

    #[inline]
    pub const fn tick_count(&self) -> u64 {
        self.tick_count
    }

    #[inline]
    pub const fn game_mode(&self) -> GameMode {
        self.game_mode
    }

    #[inline]
    pub const fn mutator_config(&self) -> &MutatorConfig {
        &self.mutator_config
    }

    pub fn set_ball_state(&mut self, ball_state: BallState) {
        self.ball
            .set_state(self.bullet_world.ball_mut(), ball_state);
    }

    pub const fn get_ball_state(&self) -> &BallState {
        &self.ball.state
    }

    /// Returns the events generated during the last stepped tick
    pub fn get_last_step_events(&self) -> &[BallHitWorldEvent] {
        self.events.events()
    }
}
