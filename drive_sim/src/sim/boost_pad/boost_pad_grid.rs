use glam::Vec3A;

use crate::{
    BoostPad, BoostPadConfig, CarBodyConfig, CarState, MutatorConfig,
    consts::boost_pads,
    shared::{Aabb, bvh},
};

struct MembershipProcessor<'a> {
    all_pads: &'a mut [BoostPad],
    pad_idx: Option<usize>,
    car_pos: Vec3A,
}

impl bvh::ProcessNode for MembershipProcessor<'_> {
    fn process_node(&mut self, pad_idx: usize) {
        if self.pad_idx.is_some() {
            return; // Already found a pad to give boost from, no need to check more
        }

        let pad = &mut self.all_pads[pad_idx];
        if overlaps_values(pad.config().pos, pad.cyl_radius, self.car_pos) {
            pad.touching = true;
            self.pad_idx = Some(pad_idx);
        }
    }
}

fn overlaps_values(pad_pos: Vec3A, radius: f32, pos: Vec3A) -> bool {
    let dist_sq_2d = pad_pos.truncate().distance_squared(pos.truncate());
    dist_sq_2d < radius.powi(2) && (pos.z - pad_pos.z).abs() <= boost_pads::CYL_HEIGHT
}

fn hitbox_overlaps_pad_box(pad_pos: Vec3A, box_radius: f32, car_state: &CarState) -> bool {
    // The arena hardcodes the Octane body config.
    let config = CarBodyConfig::OCTANE;
    let rot = car_state.phys.rot_mat;
    let center = car_state.phys.pos + rot * config.hitbox_pos_offset;
    let car_axes = [rot.x_axis, rot.y_axis, rot.z_axis];
    let car_half = config.hitbox_size * 0.5;
    let pad_half = Vec3A::new(box_radius, box_radius, boost_pads::BOX_HEIGHT);
    let delta = center - pad_pos;
    let world_axes = [Vec3A::X, Vec3A::Y, Vec3A::Z];

    for axis in world_axes
        .into_iter()
        .chain(car_axes)
        .chain(car_axes.into_iter().flat_map(|car_axis| {
            world_axes
                .into_iter()
                .map(move |world_axis| car_axis.cross(world_axis))
        }))
    {
        if axis.length_squared() <= f32::EPSILON {
            continue;
        }

        let pad_radius = pad_half.dot(Vec3A::new(
            world_axes[0].dot(axis).abs(),
            world_axes[1].dot(axis).abs(),
            world_axes[2].dot(axis).abs(),
        ));
        let car_radius = car_half.dot(Vec3A::new(
            car_axes[0].dot(axis).abs(),
            car_axes[1].dot(axis).abs(),
            car_axes[2].dot(axis).abs(),
        ));
        if delta.dot(axis).abs() > pad_radius + car_radius {
            return false;
        }
    }

    true
}

#[derive(Debug, Clone)]
pub struct BoostPadGrid {
    pub bvh_tree: bvh::Tree,
    pub all_pads: Box<[BoostPad]>,
    pub max_pad_z: f32,
    /// The only car can touch one pad at a time.
    touching_pad: Option<usize>,
}

impl BoostPadGrid {
    pub fn new(pad_configs: &[BoostPadConfig], mutator_config: &MutatorConfig) -> Self {
        assert!(!pad_configs.is_empty());

        let mut all_pads: Box<[BoostPad]> = pad_configs
            .iter()
            .map(|&pad_config| BoostPad::new(pad_config, mutator_config))
            .collect();

        // Sort them to match RLBot/RLGym ordering
        all_pads.sort_by(|a, b| {
            let a_pos = a.config.pos;
            let b_pos = b.config.pos;
            match a_pos.y.total_cmp(&b_pos.y) {
                std::cmp::Ordering::Equal => a_pos.x.total_cmp(&b_pos.x),
                other => other,
            }
        });

        let all_aabb = {
            let mut all_aabb_accum: Option<Aabb> = None;

            for pad in &all_pads {
                let pad_aabb = pad.aabb();
                if let Some(all_aabb) = all_aabb_accum {
                    all_aabb_accum = Some(all_aabb + pad_aabb);
                } else {
                    all_aabb_accum = Some(pad_aabb);
                }
            }
            all_aabb_accum.unwrap()
        };

        let mut aabb_nodes = Vec::with_capacity(all_pads.len());
        for (i, pad) in all_pads.iter().enumerate() {
            aabb_nodes.push(bvh::Node {
                aabb: pad.aabb(),
                node_type: bvh::BvhNodeType::Leaf { leaf_idx: i },
            });
        }

        let bvh_tree = bvh::Tree::build(all_aabb, &mut aabb_nodes);

        Self {
            bvh_tree,
            all_pads,
            max_pad_z: all_aabb.max.z,
            touching_pad: None,
        }
    }

    pub(crate) fn update_membership(&mut self, car_state: &CarState) {
        let car_pos = car_state.phys.pos;

        if let Some(pad_idx) = self.touching_pad {
            let pad = &self.all_pads[pad_idx];
            if overlaps_values(pad.config().pos, pad.cyl_radius, car_pos)
                || hitbox_overlaps_pad_box(pad.config().pos, pad.box_radius, car_state)
            {
                return; // Membership is unchanged.
            }

            self.all_pads[pad_idx].touching = false;
            self.touching_pad = None;
        }

        if car_pos.z > self.max_pad_z {
            return;
        }

        let car_aabb = Aabb::new(car_pos, car_pos);
        let mut processor = MembershipProcessor {
            all_pads: &mut self.all_pads,
            pad_idx: None,
            car_pos,
        };

        self.bvh_tree
            .report_aabb_overlapping_node(&mut processor, &car_aabb);
        self.touching_pad = processor.pad_idx;
    }

    /// If boost was collected, returns the pad index
    pub(crate) fn maybe_give_car_boost(
        &mut self,
        car_state: &mut CarState,
        tick_count: u64,
        tick_time: f32,
        max_boost: f32,
    ) -> Option<usize> {
        if car_state.boost >= max_boost {
            return None; // Already full on boost
        }

        let pad_idx = self.touching_pad?;
        let pad = &mut self.all_pads[pad_idx];
        if let Some(last_give_tick_count) = pad.gave_boost_tick_count
            && ((tick_count as i64 - last_give_tick_count) as f32 * tick_time) < pad.max_cooldown
        {
            return None;
        }

        car_state.boost = (car_state.boost + pad.boost_amount).min(max_boost);
        pad.gave_boost_tick_count = Some(tick_count as i64);
        Some(pad_idx)
    }

    /// Restore a pad's membership flag from a snapshot.
    pub(crate) fn set_touching(&mut self, pad_idx: usize, touching: bool) {
        self.all_pads[pad_idx].touching = touching;

        if touching {
            if let Some(previous) = self.touching_pad
                && previous != pad_idx
            {
                self.all_pads[previous].touching = false;
            }
            self.touching_pad = Some(pad_idx);
        } else if self.touching_pad == Some(pad_idx) {
            self.touching_pad = None;
        }
    }
}
