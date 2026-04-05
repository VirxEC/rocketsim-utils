use std::f32::consts::TAU;

use glam::{Vec2, Vec3A};

use crate::{
    BallState, GameMode, MutatorConfig,
    bullet::{
        collision::shapes::sphere_shape::SphereShape,
        dynamics::sphere_rigid_body::{SphereRigidBody, SphereRigidBodyConstructionInfo},
        linear_math::angle::Angle,
    },
    consts::{UU_TO_BT, dropshot, heatseeker},
    sim::consts,
};

pub(crate) struct Ball {
    pub state: BallState,
    pub ground_stick_applied: bool,
    pub vel_impulse_cache: Option<Vec3A>,
}

impl Ball {
    fn make_ball_collision_shape(
        game_mode: GameMode,
        mutator_config: &MutatorConfig,
    ) -> (SphereShape, Vec3A) {
        if game_mode == GameMode::Snowday {
            todo!()
        } else {
            let shape = SphereShape::new(mutator_config.ball_radius * UU_TO_BT);
            let local_inertia = shape.calculate_local_inertia(mutator_config.ball_mass);

            (shape, local_inertia)
        }
    }

    pub fn new(game_mode: GameMode, mutator_config: &MutatorConfig) -> (Self, SphereRigidBody) {
        let (collision_shape, local_inertia) =
            Self::make_ball_collision_shape(game_mode, mutator_config);

        let mut info =
            SphereRigidBodyConstructionInfo::new(mutator_config.ball_mass, collision_shape);
        info.start_world_trans.z = consts::ball::REST_Z * UU_TO_BT;
        info.local_inertia = local_inertia;
        info.linear_damping = mutator_config.ball_drag;

        let coefs = if game_mode == GameMode::Snowday {
            consts::snowday::PUCK_COEFS
        } else {
            consts::ball::COEFS
        };
        info.friction = coefs.friction;
        info.restitution = coefs.restitution;

        (
            Self {
                state: BallState::DEFAULT,
                ground_stick_applied: false,
                vel_impulse_cache: None,
            },
            SphereRigidBody::new(info),
        )
    }

    pub fn set_state(&mut self, rb: &mut SphereRigidBody, state: BallState) {
        rb.set_world_trans(state.phys.pos * UU_TO_BT);

        rb.set_lin_vel(state.phys.vel * UU_TO_BT);
        rb.set_ang_vel(state.phys.ang_vel);
        self.state = state;
    }

    pub(crate) fn pre_tick_update(&mut self, rb: &mut SphereRigidBody, game_mode: GameMode) {
        match game_mode {
            GameMode::Heatseeker => {
                if self.state.hs_info.y_target_dir == 0 {
                    return;
                }

                let vel_angle = Angle::from(self.state.vel);

                // Determine angle to goal
                let goal_target_pos = Vec3A::new(
                    0.0,
                    heatseeker::TARGET_Y * f32::from(self.state.hs_info.y_target_dir),
                    heatseeker::TARGET_Z,
                );
                let angle_to_goal = Angle::from(goal_target_pos - self.state.phys.pos);

                // Find difference between target angle and current angle
                let delta_angle = angle_to_goal - vel_angle;

                // Determine speed ratio
                let cur_speed = self.state.phys.vel.length();
                let speed_ratio = cur_speed / heatseeker::MAX_SPEED;

                let mut new_angle = vel_angle;
                let base_interp_factor = speed_ratio * consts::TICK_TIME;
                new_angle.yaw +=
                    delta_angle.yaw * base_interp_factor * heatseeker::HORIZONTAL_BLEND;
                new_angle.pitch +=
                    delta_angle.pitch * base_interp_factor * heatseeker::VERTICAL_BLEND;
                new_angle.yaw = new_angle.yaw.rem_euclid(TAU);
                new_angle.pitch = new_angle
                    .pitch
                    .clamp(-heatseeker::MAX_TURN_PITCH, heatseeker::MAX_TURN_PITCH);
                new_angle.normalize_fix();

                // Limit pitch
                new_angle.pitch = new_angle
                    .pitch
                    .clamp(-heatseeker::MAX_TURN_PITCH, heatseeker::MAX_TURN_PITCH);

                // Apply aggressive UE3 rotator rounding
                // (This is suprisingly important for accuracy)
                new_angle = new_angle.round_ue3();

                // Determine new interpolated speed
                let new_speed = cur_speed
                    + (self.state.hs_info.cur_target_speed - cur_speed) * heatseeker::SPEED_BLEND;

                // Update velocity
                let new_dir = new_angle.get_forward_vec();
                let new_vel = new_dir * new_speed;
                rb.set_lin_vel(new_vel * UU_TO_BT);

                self.state.hs_info.time_since_hit += consts::TICK_TIME;
            }
            GameMode::Snowday => self.ground_stick_applied = false,
            GameMode::Dropshot | GameMode::Hoops => {
                // Launch ball after a short delay on kickoff
                let is_dropshot = game_mode == GameMode::Dropshot;

                let launch_delay = if is_dropshot {
                    dropshot::BALL_LAUNCH_DELAY
                } else {
                    consts::ball::HOOPS_LAUNCH_DELAY
                };

                let cur_kickoff_time =
                    self.state.tick_count_since_kickoff as f32 * consts::TICK_TIME;
                let prev_kickoff_time = cur_kickoff_time - consts::TICK_TIME;

                if prev_kickoff_time < launch_delay && cur_kickoff_time >= launch_delay {
                    // Launch triggered
                    // Make sure the ball is frozen at the kickoff X and Y
                    if self.state.phys.vel == Vec3A::ZERO
                        && self.state.phys.ang_vel == Vec3A::ZERO
                        && self.state.phys.pos.truncate() == Vec2::ZERO
                    {
                        // Apply the force
                        let launch_vel_z = if is_dropshot {
                            dropshot::BALL_LAUNCH_Z_VEL
                        } else {
                            consts::ball::HOOPS_LAUNCH_Z_VEL
                        };

                        rb.apply_central_impulse(
                            Vec3A::new(0.0, 0.0, launch_vel_z) * rb.get_mass() * UU_TO_BT,
                        );
                    }
                }
            }
            _ => {}
        }
    }

    pub(crate) fn finish_physics_tick(
        &mut self,
        rb: &mut SphereRigidBody,
        mutator_config: &MutatorConfig,
    ) {
        if let Some(vel_impulse_cache) = self.vel_impulse_cache {
            rb.lin_vel += vel_impulse_cache * UU_TO_BT;
            self.vel_impulse_cache = None;
        }

        let ball_max_speed_bt = mutator_config.ball_max_speed * UU_TO_BT;
        let lin_vel_sq = rb.lin_vel.length_squared();
        if lin_vel_sq > ball_max_speed_bt * ball_max_speed_bt {
            rb.lin_vel = rb.lin_vel * (1.0 / lin_vel_sq.sqrt()) * ball_max_speed_bt;
        }

        let ang_vel_sq = rb.ang_vel.length_squared();
        if ang_vel_sq > consts::ball::MAX_ANG_SPEED * consts::ball::MAX_ANG_SPEED {
            rb.ang_vel = rb.ang_vel * (1.0 / ang_vel_sq.sqrt()) * consts::ball::MAX_ANG_SPEED;
        }

        self.state.phys.vel = rb.lin_vel * consts::BT_TO_UU;
        self.state.phys.ang_vel = rb.ang_vel;

        self.state.phys.pos = rb.get_world_trans() * consts::BT_TO_UU;

        self.state.tick_count_since_kickoff += 1;
    }

    pub fn on_world_hit(&mut self, normal: Vec3A, game_mode: GameMode) {
        match game_mode {
            GameMode::Heatseeker => {
                const ARENA_EXTENT: Vec3A = consts::arena::get_aabb(GameMode::Soccar).max;
                if self.state.hs_info.y_target_dir == 0 {
                    return;
                }

                let y_target_dir = f32::from(self.state.hs_info.y_target_dir);
                let rel_normal_y = normal.y * y_target_dir;
                let rel_y = self.state.phys.pos.y * y_target_dir;
                if rel_normal_y <= -heatseeker::WALL_BOUNCE_CHANGE_Y_NORMAL
                    && rel_y >= ARENA_EXTENT.y - heatseeker::WALL_BOUNCE_CHANGE_Y_THRESH
                {
                    // We hit far enough to change direction
                    self.state.hs_info.y_target_dir *= -1;

                    let goal_target_pos = Vec3A::new(
                        0.0,
                        heatseeker::TARGET_Y * y_target_dir,
                        heatseeker::TARGET_Z,
                    );

                    // Add wall bounce impulse
                    let dir_to_goal = (goal_target_pos - self.state.phys.pos).normalize_or_zero();

                    let bounce_dir = dir_to_goal * (1.0 - heatseeker::WALL_BOUNCE_UP_FRAC)
                        + Vec3A::Z * heatseeker::WALL_BOUNCE_UP_FRAC;
                    let bounce_impulse = bounce_dir
                        * self.state.phys.vel.length()
                        * heatseeker::WALL_BOUNCE_FORCE_SCALE;

                    if let Some(vel_impulse_cache) = self.vel_impulse_cache.as_mut() {
                        *vel_impulse_cache += bounce_impulse * UU_TO_BT;
                    } else {
                        self.vel_impulse_cache = Some(bounce_impulse * UU_TO_BT);
                    }
                }
            }
            GameMode::Snowday => todo!(),
            _ => {}
        }
    }
}
