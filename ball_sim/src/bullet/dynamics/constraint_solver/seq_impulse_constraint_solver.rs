use glam::Vec3A;

use super::{contact_solver_info, solver_body::SolverBody, solver_constraint::SolverConstraint};
use crate::bullet::{
    collision::narrowphase::{
        manifold_point::ManifoldPoint,
        persistent_manifold::{MANIFOLD_CACHE_SIZE, PersistentManifold},
    },
    dynamics::sphere_rigid_body::SphereRigidBody,
    linear_math::plane_space_1,
};

/// Use Bullet's linear slop to classify shallow support contacts.
/// Treat deeper contacts as impacts.
const SPECIAL_LINEAR_SLOP: f32 = 0.04;

/// Blend samples only for upward-facing support contacts.
/// Use facet responses for steep contacts.
const SUPPORT_NORMAL_MIN_Z: f32 = 0.866;

#[derive(Clone, Copy, Debug)]
struct SpecialContact {
    obj_idx: usize,
    static_obj_idx: usize,
    pos_world_on_a: Vec3A,
    normal_world_on_b: Vec3A,
    /// Save the closest-feature normal before edge adjustment.
    raw_normal_world_on_b: Vec3A,
    /// Store the lever arm from the body center to the contact point.
    lever_arm: Vec3A,
    distance: f32,
    friction: f32,
    restitution: f32,
}

impl SpecialContact {
    fn is_penetrating(&self) -> bool {
        self.distance < 0.0
    }

    /// Return the contact speed along the normal. Negative means closing.
    fn approach_speed(&self, lin_vel: Vec3A, ang_vel: Vec3A) -> f32 {
        let contact_vel = lin_vel + ang_vel.cross(self.lever_arm);
        self.normal_world_on_b.dot(contact_vel)
    }
}

fn special_contact_from_point(
    cp: &ManifoldPoint,
    rel_pos: Vec3A,
    static_obj_idx: usize,
) -> SpecialContact {
    // Ball-only: the ball is the single dynamic body, so it owns every
    // special contact. `static_obj_idx` is the persistent manifold slot,
    // which identifies the static pair within a tick.
    SpecialContact {
        obj_idx: 0,
        static_obj_idx,
        pos_world_on_a: cp.pos_world_on_a,
        normal_world_on_b: cp.normal_world_on_b,
        raw_normal_world_on_b: cp.raw_normal_world_on_b,
        lever_arm: rel_pos,
        distance: cp.distance_1,
        friction: cp.combined_friction,
        restitution: cp.combined_restitution,
    }
}

/// Select the deepest penetrating sample.
/// Otherwise, select the fastest-closing sample.
fn dominant_contact(cluster: &[SpecialContact], lin_vel: Vec3A, ang_vel: Vec3A) -> &SpecialContact {
    let mut best: Option<&SpecialContact> = None;
    for contact in cluster {
        let take = match best {
            None => true,
            Some(b) => {
                if contact.is_penetrating() == b.is_penetrating() {
                    if contact.is_penetrating() {
                        contact.distance < b.distance
                    } else {
                        let v_contact = contact.approach_speed(lin_vel, ang_vel);
                        let v_best = b.approach_speed(lin_vel, ang_vel);
                        v_contact < v_best
                    }
                } else {
                    // Prefer penetrating samples.
                    contact.is_penetrating()
                }
            }
        };
        if take {
            best = Some(contact);
        }
    }
    best.unwrap()
}

/// Group nearby samples from one dynamic body as one physical touch.
const SPECIAL_CONTACT_CLUSTER_LEVER_FRACTION: f32 = 0.02;

#[derive(Clone, Debug)]
pub struct SeqImpulseConstraintSolver {
    solver_body: SolverBody,
    contact_constraint: Option<(SolverConstraint, SolverConstraint)>,
    tmp_special_contact_pool: Vec<SpecialContact>,
    tmp_special_contact_group_ends: Vec<usize>,
    tmp_special_resolved_touches: Vec<SpecialContact>,
    tmp_special_cluster_pool: Vec<SpecialContact>,
    tmp_special_cluster_ranges: Vec<(usize, usize)>,
    tmp_special_representatives: Vec<SpecialContact>,
    tmp_special_shallow_reduced: Vec<SpecialContact>,
    tmp_special_shallow_edge_group_pool: Vec<SpecialContact>,
    tmp_special_shallow_edge_group_ranges: Vec<(usize, usize)>,
    tmp_special_shallow_deduplicated: Vec<SpecialContact>,
}

impl Default for SeqImpulseConstraintSolver {
    fn default() -> Self {
        Self {
            solver_body: SolverBody::DEFAULT,
            contact_constraint: None,
            tmp_special_contact_pool: Vec::new(),
            tmp_special_contact_group_ends: Vec::new(),
            tmp_special_resolved_touches: Vec::new(),
            tmp_special_cluster_pool: Vec::new(),
            tmp_special_cluster_ranges: Vec::new(),
            tmp_special_representatives: Vec::new(),
            tmp_special_shallow_reduced: Vec::new(),
            tmp_special_shallow_edge_group_pool: Vec::new(),
            tmp_special_shallow_edge_group_ranges: Vec::new(),
            tmp_special_shallow_deduplicated: Vec::new(),
        }
    }
}

impl SeqImpulseConstraintSolver {
    pub fn solve_group(
        &mut self,
        ball_obj: &mut SphereRigidBody,
        manifolds: &mut [PersistentManifold],
        active_manifold_idcs: &mut Vec<usize>,
        time_step: f32,
    ) {
        self.solve_group_setup(ball_obj, manifolds, active_manifold_idcs, time_step);
        self.solve_group_iterations();
        self.solve_group_finish(ball_obj, time_step);
    }

    fn solve_group_setup(
        &mut self,
        ball_obj: &mut SphereRigidBody,
        manifolds: &mut [PersistentManifold],
        active_manifold_idcs: &mut Vec<usize>,
        time_step: f32,
    ) {
        self.setup_solver_bodies(ball_obj);
        self.tmp_special_contact_pool.clear();
        self.tmp_special_contact_group_ends.clear();
        self.tmp_special_resolved_touches.clear();

        // Iterate the persistent manifolds for this tick's active pairs
        // directly. Only `lateral_friction_dir_1` is mutated, and it is
        // recomputed from scratch on every read, so sharing the
        // persistent copy (instead of solving a per-tick clone) changes
        // no contact order, threshold, selection, or warmstart state.
        // Ball-only: every manifold point is a ball-vs-static-world
        // contact (upstream marks these `is_special` in the contact
        // tracker); process them via the special path.
        let ball_trans = ball_obj.get_world_trans();
        for &manifold_idx in active_manifold_idcs.iter() {
            let manifold = &mut manifolds[manifold_idx];
            let evicted_point = manifold.most_recently_evicted_point;
            let special_contact_start = self.tmp_special_contact_pool.len();

            for cp in &mut manifold.point_cache {
                assert!(cp.distance_1 <= manifold.contact_processing_threshold);

                let rel_pos = cp.pos_world_on_a - ball_trans;
                self.tmp_special_contact_pool
                    .push(special_contact_from_point(cp, rel_pos, manifold_idx));
            }

            let evicted_special_contact = evicted_point.map(|cp| {
                let rel_pos = cp.pos_world_on_a - ball_trans;
                special_contact_from_point(&cp, rel_pos, manifold_idx)
            });

            if let Some(evicted_contact) = evicted_special_contact {
                replace_shallower_duplicate(
                    &mut self.tmp_special_contact_pool[special_contact_start..],
                    evicted_contact,
                );
            }

            if self.tmp_special_contact_pool.len() != special_contact_start {
                self.tmp_special_contact_group_ends
                    .push(self.tmp_special_contact_pool.len());
            }
        }

        // Drop this tick's active list. Persistent manifolds keep their
        // points and warmstart impulses for the next tick.
        active_manifold_idcs.clear();

        if !self.tmp_special_contact_pool.is_empty() {
            if self.all_special_shallow() {
                self.convert_contact_special(ball_obj, time_step);
            } else {
                self.collect_survivor_clusters();
                self.convert_cluster_averages(ball_obj, time_step);
            }
        }

        self.tmp_special_contact_group_ends.clear();
    }

    fn all_special_shallow(&self) -> bool {
        self.tmp_special_contact_pool
            .iter()
            .all(|contact| !contact.is_penetrating())
    }

    /// Move each contact group into the cluster pool.
    /// Skip impact groups that duplicate an already resolved touch.
    fn collect_survivor_clusters(&mut self) {
        let Self {
            tmp_special_contact_pool: contacts,
            tmp_special_contact_group_ends: group_ends,
            tmp_special_cluster_pool: clusters,
            tmp_special_resolved_touches: resolved,
            ..
        } = self;

        clusters.clear();
        let mut group_start = 0;
        for &group_end in group_ends.iter() {
            let group = &contacts[group_start..group_end];
            let has_impact = group.iter().any(SpecialContact::is_penetrating);
            if has_impact {
                let is_duplicate_view = group
                    .iter()
                    .all(|contact| resolved.iter().any(|r| same_touch(r, contact)));
                if is_duplicate_view {
                    group_start = group_end;
                    continue;
                }

                resolved.extend_from_slice(group);
            }

            clusters.extend_from_slice(group);
            group_start = group_end;
        }
    }

    fn setup_solver_bodies(&mut self, ball_obj: &SphereRigidBody) {
        self.solver_body.update(ball_obj);
    }

    fn convert_contact_special(&mut self, ball_obj: &SphereRigidBody, time_step: f32) {
        // Reduce duplicate reports before classifying shallow support contacts.
        // Use one representative for each deep physical touch.
        let has_reduced_contacts = reduce_shallow_support_contacts(
            &self.tmp_special_contact_pool,
            &mut self.tmp_special_shallow_reduced,
            &mut self.tmp_special_shallow_edge_group_pool,
            &mut self.tmp_special_shallow_edge_group_ranges,
            &mut self.tmp_special_shallow_deduplicated,
        );

        let aggregate = {
            let classification_contacts: &[SpecialContact] = if has_reduced_contacts {
                &self.tmp_special_shallow_deduplicated
            } else {
                &self.tmp_special_contact_pool
            };
            let min_distance = classification_contacts
                .iter()
                .map(|contact| contact.distance)
                .fold(f32::MAX, f32::min);

            let all_non_penetrating = self
                .tmp_special_contact_pool
                .iter()
                .all(|contact| !contact.is_penetrating());
            // Evaluate the deep-contact condition lazily: the mean below is only
            // needed when a penetrating sample exists.
            let is_shallow_support = all_non_penetrating || {
                let mut total_classification_normal = Vec3A::ZERO;
                for contact in classification_contacts {
                    total_classification_normal += if has_reduced_contacts {
                        contact.normal_world_on_b
                    } else {
                        contact.raw_normal_world_on_b
                    };
                }

                let classification_mean =
                    total_classification_normal / classification_contacts.len() as f32;
                min_distance > -SPECIAL_LINEAR_SLOP && classification_mean.z >= SUPPORT_NORMAL_MIN_Z
            };
            if is_shallow_support {
                let aggregate_contacts = classification_contacts;
                let mut total_normal = Vec3A::ZERO;
                let mut total_lever_len = 0.0;
                for contact in aggregate_contacts {
                    total_normal += if has_reduced_contacts {
                        contact.normal_world_on_b
                    } else {
                        contact.raw_normal_world_on_b
                    };
                    total_lever_len += contact.lever_arm.length();
                }

                let num_samples = aggregate_contacts.len() as f32;
                SpecialAggregate {
                    normal_world_on_b: (total_normal / num_samples).normalize(),
                    lever_len: total_lever_len / num_samples,
                    min_distance,
                    obj_idx: self.tmp_special_contact_pool[0].obj_idx,
                }
            } else {
                // Group deep samples by physical touch.
                // Use one representative for each touch.
                self.tmp_special_cluster_pool.clear();
                self.tmp_special_cluster_ranges.clear();
                self.tmp_special_representatives.clear();
                let num_contacts = self.tmp_special_contact_pool.len();
                self.tmp_special_cluster_pool.reserve(num_contacts);
                self.tmp_special_cluster_ranges.reserve(num_contacts);
                self.tmp_special_representatives.reserve(num_contacts);

                for contact in &self.tmp_special_contact_pool {
                    let matching_cluster =
                        self.tmp_special_cluster_ranges.iter().enumerate().find_map(
                            |(cluster_idx, &(start, end))| {
                                if self.tmp_special_cluster_pool[start..end]
                                    .iter()
                                    .any(|member| same_touch(member, contact))
                                {
                                    Some(cluster_idx)
                                } else {
                                    None
                                }
                            },
                        );

                    if let Some(cluster_idx) = matching_cluster {
                        let insert_at = self.tmp_special_cluster_ranges[cluster_idx].1;
                        self.tmp_special_cluster_pool.insert(insert_at, *contact);
                        for (range_idx, range) in self
                            .tmp_special_cluster_ranges
                            .iter_mut()
                            .enumerate()
                            .skip(cluster_idx)
                        {
                            range.1 += 1;
                            if range_idx > cluster_idx {
                                range.0 += 1;
                            }
                        }
                    } else {
                        let start = self.tmp_special_cluster_pool.len();
                        self.tmp_special_cluster_pool.push(*contact);
                        self.tmp_special_cluster_ranges.push((start, start + 1));
                    }
                }

                // Use the touched body's velocities for every sample in the cluster.
                let (lin_vel, ang_vel) = (self.solver_body.lin_vel, self.solver_body.ang_vel);
                for cluster_idx in 0..self.tmp_special_cluster_ranges.len() {
                    let representative = {
                        let (start, end) = self.tmp_special_cluster_ranges[cluster_idx];
                        let cluster = &self.tmp_special_cluster_pool[start..end];
                        dominant_contact(cluster, lin_vel, ang_vel)
                    };
                    self.tmp_special_representatives.push(*representative);
                }

                // Ignore stationary touches when another touch approaches.
                const STATIONARY_SPEED: f32 = 0.01;
                let any_approaching = self
                    .tmp_special_representatives
                    .iter()
                    .any(|contact| contact.approach_speed(lin_vel, ang_vel) < -STATIONARY_SPEED);
                if any_approaching {
                    self.tmp_special_representatives.retain(|contact| {
                        contact.approach_speed(lin_vel, ang_vel) < -STATIONARY_SPEED
                    });
                }

                let mut total_normal = Vec3A::ZERO;
                let mut total_lever_len = 0.0;
                for representative in &self.tmp_special_representatives {
                    total_normal += representative.normal_world_on_b;
                    total_lever_len += representative.lever_arm.length();
                }

                let num_representatives = self.tmp_special_representatives.len() as f32;
                SpecialAggregate {
                    normal_world_on_b: (total_normal / num_representatives).normalize(),
                    lever_len: total_lever_len / num_representatives,
                    min_distance: self
                        .tmp_special_representatives
                        .iter()
                        .map(|contact| contact.distance)
                        .fold(f32::MAX, f32::min),
                    obj_idx: self.tmp_special_representatives[0].obj_idx,
                }
            }
        };

        let first_contact = self.tmp_special_contact_pool[0];
        let contact = SpecialContact {
            normal_world_on_b: aggregate.normal_world_on_b,
            lever_arm: aggregate.normal_world_on_b * -aggregate.lever_len,
            distance: aggregate.min_distance,
            friction: first_contact.friction,
            restitution: first_contact.restitution,
            obj_idx: aggregate.obj_idx,
            ..first_contact
        };

        self.push_special_row::<false>(ball_obj, &contact, time_step);
    }

    /// Create one averaged synthetic row for each dynamic body.
    /// Read survivors from the cluster pool. Copy each sample by value.
    /// Hold no pool borrow across `push_special_row`.
    fn convert_cluster_averages(&mut self, ball_obj: &SphereRigidBody, time_step: f32) {
        let num_clusters = self.tmp_special_cluster_pool.len();
        'clusters: for i in 0..num_clusters {
            let contact = &self.tmp_special_cluster_pool[i];
            for other_contact in &self.tmp_special_cluster_pool[0..i] {
                if other_contact.obj_idx == contact.obj_idx {
                    continue 'clusters;
                }
            }

            let mut total_normal = Vec3A::ZERO;
            let mut total_lever_len = 0.0;
            let mut total_friction = 0.0;
            let mut total_restitution = 0.0;
            let mut min_distance = f32::MAX;
            let mut count = 0u32;
            let mut first = contact;

            let obj_idx = contact.obj_idx;
            for contact in &self.tmp_special_cluster_pool {
                if contact.obj_idx != obj_idx {
                    continue;
                }

                total_normal += contact.normal_world_on_b;
                total_lever_len += contact.lever_arm.length();
                total_friction += contact.friction;
                total_restitution += contact.restitution;
                min_distance = min_distance.min(contact.distance);
                count += 1;
                first = contact;
            }

            let num_samples = count as f32;
            let mean_normal = (total_normal / num_samples).normalize();
            let contact = SpecialContact {
                normal_world_on_b: mean_normal,
                lever_arm: mean_normal * -(total_lever_len / num_samples),
                distance: min_distance,
                friction: total_friction / num_samples,
                restitution: total_restitution / num_samples,
                obj_idx,
                ..*first
            };

            self.push_special_row::<true>(ball_obj, &contact, time_step);
        }
    }

    fn push_special_row<const STANDARD_RESTITUTION: bool>(
        &mut self,
        body: &SphereRigidBody,
        contact: &SpecialContact,
        time_step: f32,
    ) {
        let relaxation = contact_solver_info::SOR;

        let normal_world_on_b = contact.normal_world_on_b;
        let rel_pos1 = contact.lever_arm;
        let penetration = contact.distance;

        let inv_time_step = 1.0 / time_step;
        let erp = contact_solver_info::ERP_2;

        let torque_axis_0 = rel_pos1.cross(normal_world_on_b);
        let angular_component_a = body.inv_inertia_local * torque_axis_0;

        let denom = {
            let vec = angular_component_a.cross(rel_pos1);
            body.inv_mass + normal_world_on_b.dot(vec)
        };
        let jac_diag_ab_inv = relaxation / denom;

        let (contact_normal_1, rel_pos1_cross_normal) = (normal_world_on_b, torque_axis_0);

        let vel = body.get_vel_in_local_point(rel_pos1);
        let rel_vel = normal_world_on_b.dot(vel);

        let restitution = if STANDARD_RESTITUTION
            || rel_vel.abs() >= contact_solver_info::SPECIAL_RESTITUTION_VELOCITY_THRESHOLD
        {
            SolverConstraint::restitution_curve(rel_vel, contact.restitution)
        } else {
            0.0
        };

        let external_force_impulse_a = self.solver_body.external_force_impulse;

        let rel_vel = contact_normal_1.dot(self.solver_body.lin_vel + external_force_impulse_a)
            + rel_pos1_cross_normal.dot(self.solver_body.ang_vel);

        let positional_error = if penetration > 0.0 {
            0.0
        } else {
            -penetration * erp * inv_time_step
        };

        let vel_error = restitution - rel_vel;

        let penetration_impulse = positional_error * jac_diag_ab_inv;
        let vel_impulse = vel_error * jac_diag_ab_inv;

        let (rhs, rhs_penetration) =
            if penetration > contact_solver_info::SPLIT_IMPULSE_PENETRATION_THRESHOLD {
                (penetration_impulse + vel_impulse, 0.0)
            } else {
                (vel_impulse, penetration_impulse)
            };

        let contact_constraint = SolverConstraint {
            angular_component_a,
            jac_diag_ab_inv,
            contact_normal_1,
            rel_pos1_cross_normal,
            rhs,
            rhs_penetration,
            friction: contact.friction,
            lower_limit: 0.0,
            upper_limit: 1e10,
            ..Default::default()
        };

        let vel = self.solver_body.get_vel_in_local_point_no_delta(rel_pos1);
        let rel_vel = normal_world_on_b.dot(vel);

        let mut lateral_friction_dir_1 = vel - normal_world_on_b * rel_vel;
        let lat_rel_vel = lateral_friction_dir_1.length_squared();

        if lat_rel_vel > f32::EPSILON {
            lateral_friction_dir_1 *= 1.0 / lat_rel_vel.sqrt();
        } else {
            lateral_friction_dir_1 = plane_space_1(normal_world_on_b);
        }

        // Add the friction constraint.
        let (contact_normal_1, rel_pos1_cross_normal, angular_component_a) = {
            let torque_axis = rel_pos1.cross(lateral_friction_dir_1);

            (
                lateral_friction_dir_1,
                torque_axis,
                body.inv_inertia_local * torque_axis,
            )
        };

        let denom = {
            let vec = angular_component_a.cross(rel_pos1);
            body.inv_mass + lateral_friction_dir_1.dot(vec)
        };
        let jac_diag_ab_inv = relaxation / denom;

        let rel_vel = contact_normal_1.dot(self.solver_body.lin_vel + external_force_impulse_a)
            + rel_pos1_cross_normal.dot(self.solver_body.ang_vel);

        let vel_error = -rel_vel;
        let vel_impulse = vel_error * jac_diag_ab_inv;

        self.contact_constraint = Some((
            contact_constraint,
            SolverConstraint {
                contact_normal_1,
                rel_pos1_cross_normal,
                angular_component_a,
                jac_diag_ab_inv,
                rhs: vel_impulse,
                lower_limit: -contact.friction,
                upper_limit: contact.friction,
                friction: contact.friction,
                ..Default::default()
            },
        ));
    }

    fn solve_group_iterations(&mut self) {
        if let Some((contact, friction)) = self.contact_constraint.as_mut() {
            self.solver_body
                .solve_group_split_impulse_iterations(contact);

            for _ in 0..contact_solver_info::NUM_ITERATIONS {
                let least_squares_residual =
                    self.solver_body.solve_single_iteration(contact, friction);
                if least_squares_residual == 0.0 {
                    break;
                }
            }
        }
    }

    fn solve_group_finish(&mut self, body: &mut SphereRigidBody, time_step: f32) {
        self.solver_body.lin_vel += self.solver_body.delta_lin_vel;
        self.solver_body.ang_vel += self.solver_body.delta_ang_vel;

        // Skip the transform write-back unless split impulse moved the body.
        // Without push motion the body transform is already current, so there
        // is nothing to write back. Reload it from the body itself: bodies
        // are untouched between solver setup and write-back.
        if self.solver_body.push_vel.length_squared() != 0.0
            || self.solver_body.turn_vel.length_squared() != 0.0
        {
            body.set_world_trans(body.get_world_trans() + self.solver_body.push_vel * time_step);
        }

        body.set_lin_vel(self.solver_body.lin_vel + self.solver_body.external_force_impulse);
        body.set_ang_vel(self.solver_body.ang_vel);

        self.contact_constraint = None;
    }
}

/// Store the aggregate response for one special contact region.
struct SpecialAggregate {
    normal_world_on_b: Vec3A,
    lever_len: f32,
    min_distance: f32,
    obj_idx: usize,
}

/// Return whether two samples belong to one physical touch.
fn same_touch(a: &SpecialContact, b: &SpecialContact) -> bool {
    if a.obj_idx != b.obj_idx {
        return false;
    }

    let delta = (a.pos_world_on_a - b.pos_world_on_a).abs();
    let dx = delta.x.min((a.pos_world_on_a.x + b.pos_world_on_a.x).abs());
    let dy = delta.y.min((a.pos_world_on_a.y + b.pos_world_on_a.y).abs());

    let tolerance = a.lever_arm.length() * SPECIAL_CONTACT_CLUSTER_LEVER_FRACTION;
    dx * dx + dy * dy + delta.z * delta.z <= tolerance * tolerance
}

const SPECIAL_NORMAL_ADJUSTMENT_EPSILON: f32 = 1e-4;

fn same_contact_position(a: &SpecialContact, b: &SpecialContact) -> bool {
    let tolerance =
        a.lever_arm.length().max(b.lever_arm.length()) * SPECIAL_CONTACT_CLUSTER_LEVER_FRACTION;
    (a.pos_world_on_a - b.pos_world_on_a).length_squared() <= tolerance * tolerance
}

fn same_adjusted_normal(a: &SpecialContact, b: &SpecialContact) -> bool {
    a.normal_world_on_b.dot(b.normal_world_on_b) >= 1.0 - SPECIAL_NORMAL_ADJUSTMENT_EPSILON
}

fn replace_shallower_duplicate(
    special_contacts: &mut [SpecialContact],
    evicted_contact: SpecialContact,
) {
    if special_contacts.len() != MANIFOLD_CACHE_SIZE
        || special_contacts
            .iter()
            .any(|contact| same_adjusted_normal(contact, &evicted_contact))
    {
        return;
    }

    for first_idx in 0..special_contacts.len() {
        for second_idx in first_idx + 1..special_contacts.len() {
            if !same_adjusted_normal(&special_contacts[first_idx], &special_contacts[second_idx]) {
                continue;
            }

            let replacement_idx =
                if special_contacts[first_idx].distance > special_contacts[second_idx].distance {
                    first_idx
                } else {
                    second_idx
                };
            special_contacts[replacement_idx] = evicted_contact;
            return;
        }
    }
}

fn is_edge_adjusted(contact: &SpecialContact) -> bool {
    contact.raw_normal_world_on_b.dot(contact.normal_world_on_b)
        < 1.0 - SPECIAL_NORMAL_ADJUSTMENT_EPSILON
}

/// Reduce duplicate reports before aggregating shallow contacts.
/// Keep one representative for each physical feature.
fn reduce_shallow_support_contacts(
    special_contacts: &[SpecialContact],
    reduced: &mut Vec<SpecialContact>,
    edge_group_contacts: &mut Vec<SpecialContact>,
    edge_group_ranges: &mut Vec<(usize, usize)>,
    deduplicated: &mut Vec<SpecialContact>,
) -> bool {
    reduced.clear();
    edge_group_contacts.clear();
    edge_group_ranges.clear();
    deduplicated.clear();

    if !special_contacts.iter().any(is_edge_adjusted) {
        return false;
    }

    reduced.reserve(special_contacts.len());
    edge_group_contacts.reserve(special_contacts.len());
    edge_group_ranges.reserve(special_contacts.len());
    deduplicated.reserve(special_contacts.len());

    for contact in special_contacts {
        if is_edge_adjusted(contact) {
            continue;
        }

        let duplicate = reduced.iter().any(|existing| {
            existing.static_obj_idx == contact.static_obj_idx
                && same_contact_position(existing, contact)
                && same_adjusted_normal(existing, contact)
        });
        if !duplicate {
            reduced.push(*contact);
        }
    }

    for contact in special_contacts {
        if !is_edge_adjusted(contact) {
            continue;
        }

        let matching_group =
            edge_group_ranges
                .iter()
                .enumerate()
                .find_map(|(group_idx, &(start, _))| {
                    if edge_group_contacts[start].static_obj_idx == contact.static_obj_idx
                        && same_contact_position(&edge_group_contacts[start], contact)
                    {
                        Some(group_idx)
                    } else {
                        None
                    }
                });

        if let Some(group_idx) = matching_group {
            let insert_at = edge_group_ranges[group_idx].1;
            edge_group_contacts.insert(insert_at, *contact);
            for (range_idx, range) in edge_group_ranges.iter_mut().enumerate().skip(group_idx) {
                range.1 += 1;
                if range_idx > group_idx {
                    range.0 += 1;
                }
            }
        } else {
            let start = edge_group_contacts.len();
            edge_group_contacts.push(*contact);
            edge_group_ranges.push((start, start + 1));
        }
    }

    for &(start, end) in edge_group_ranges.iter() {
        let group = &edge_group_contacts[start..end];
        let mut adjusted_normal = Vec3A::ZERO;
        for contact in group {
            adjusted_normal += contact.normal_world_on_b;
        }
        adjusted_normal = adjusted_normal.normalize();

        let all_normals_agree = group.iter().all(|contact| {
            contact.normal_world_on_b.dot(adjusted_normal)
                >= 1.0 - SPECIAL_NORMAL_ADJUSTMENT_EPSILON
        });

        if all_normals_agree {
            let mut representative = group[0];
            representative.normal_world_on_b = adjusted_normal;
            reduced.push(representative);
            continue;
        }

        let has_stable_owner = group.iter().all(|edge| {
            special_contacts.iter().any(|facet| {
                !is_edge_adjusted(facet)
                    && facet.static_obj_idx == edge.static_obj_idx
                    && same_adjusted_normal(facet, edge)
            })
        });
        if !has_stable_owner {
            let representative = group
                .iter()
                .min_by(|a, b| a.distance.total_cmp(&b.distance))
                .copied()
                .unwrap();
            reduced.push(representative);
        }
    }

    for contact in reduced.iter() {
        let duplicate_penetrating_feature = contact.is_penetrating()
            && deduplicated.iter().any(|existing| {
                existing.is_penetrating()
                    && existing.static_obj_idx != contact.static_obj_idx
                    && same_contact_position(existing, contact)
                    && same_adjusted_normal(existing, contact)
            });
        if !duplicate_penetrating_feature {
            deduplicated.push(*contact);
        }
    }

    true
}

#[cfg(test)]
mod tests {
    use glam::Vec3A;

    use super::{SpecialContact, replace_shallower_duplicate};

    fn contact(normal: Vec3A, distance: f32, marker: f32) -> SpecialContact {
        SpecialContact {
            obj_idx: 1,
            static_obj_idx: 2,
            pos_world_on_a: Vec3A::new(marker, 0.0, 0.0),
            normal_world_on_b: normal,
            raw_normal_world_on_b: normal,
            lever_arm: Vec3A::Z,
            distance,
            friction: 0.5,
            restitution: 0.0,
        }
    }

    #[test]
    fn evicted_contact_replaces_shallower_duplicate() {
        let evicted = contact(Vec3A::Z, -0.08, 9.0);
        let mut contacts = [
            contact(Vec3A::X, -0.05, 1.0),
            contact(Vec3A::X, -0.20, 2.0),
            contact(Vec3A::Y, -0.10, 3.0),
            contact(Vec3A::new(0.0, 0.0, -1.0), -0.10, 4.0),
        ];

        replace_shallower_duplicate(&mut contacts, evicted);

        assert_eq!(contacts[0].pos_world_on_a, evicted.pos_world_on_a);
        assert_eq!(contacts[0].normal_world_on_b, evicted.normal_world_on_b);
        assert_eq!(contacts[0].distance, evicted.distance);
        assert_eq!(contacts[1].pos_world_on_a.x, 2.0);
        assert_eq!(contacts[1].normal_world_on_b, Vec3A::X);
        assert_eq!(contacts[1].distance, -0.20);
        assert_eq!(contacts[2].pos_world_on_a.x, 3.0);
        assert_eq!(contacts[3].pos_world_on_a.x, 4.0);
    }
}
