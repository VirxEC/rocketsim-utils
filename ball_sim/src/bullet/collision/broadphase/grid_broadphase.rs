use std::iter::repeat_with;

use arrayvec::ArrayVec;
use glam::{IVec3, USizeVec3, Vec3A};

use super::overlapping_pair_cache::HashedOverlappingPairCache;
use crate::{
    ArenaContactTracker,
    bullet::{
        collision::{
            dispatch::collision_dispatcher::CollisionDispatcher,
            shapes::collision_shape::CollisionShapes,
        },
        dynamics::{rigid_body::RigidBody, sphere_rigid_body::SphereRigidBody},
    },
    shared::Aabb,
};

#[derive(Clone, Copy, Debug)]
pub struct GridBroadphaseProxy {
    /// The index of the client `RigidBody` in `CollisionWorld`
    pub client_obj_idx: usize,
    pub aabb: Aabb,
    cell_idx: usize,
}

#[derive(Clone, Debug)]
struct GridCell {
    static_handles: ArrayVec<usize, 4>,
}

impl GridCell {
    fn new() -> Self {
        Self {
            static_handles: ArrayVec::new(),
        }
    }
}

#[derive(Clone, Debug)]
struct CellGrid {
    max_pos: Vec3A,
    min_pos: Vec3A,
    cell_size: f32,
    num_cells: USizeVec3,
    cells: Box<[GridCell]>,
}

impl CellGrid {
    fn get_cell_indices(&self, pos: Vec3A) -> USizeVec3 {
        let cell_idx_f = (pos - self.min_pos) / self.cell_size;
        unsafe {
            IVec3 {
                x: cell_idx_f.x.to_int_unchecked::<i32>(),
                y: cell_idx_f.y.to_int_unchecked::<i32>(),
                z: cell_idx_f.z.to_int_unchecked::<i32>(),
            }
        }
        .max(IVec3::ZERO)
        .as_usizevec3()
        .min(self.num_cells - USizeVec3::ONE)
    }

    const fn cell_indices_to_idx(&self, indices: USizeVec3) -> usize {
        indices.x * self.num_cells.y * self.num_cells.z + indices.y * self.num_cells.z + indices.z
    }

    fn get_cell_min_pos(&self, indices: USizeVec3) -> Vec3A {
        self.min_pos + indices.as_vec3a() * self.cell_size
    }

    fn update_cells_static(
        &mut self,
        proxy: &GridBroadphaseProxy,
        col_obj: &RigidBody,
        proxy_idx: usize,
    ) {
        let min = self.get_cell_indices(proxy.aabb.min.max(self.min_pos));
        let max = self.get_cell_indices(proxy.aabb.max.min(self.max_pos));

        let tri_mesh_shape = match col_obj.get_collision_shape() {
            CollisionShapes::TriangleMesh(mesh) => Some(mesh.as_ref()),
            CollisionShapes::StaticPlane(_) => None,
        };

        for i in min.x..=max.x {
            for j in min.y..=max.y {
                for k in min.z..=max.z {
                    if let Some(mesh_interface) = tri_mesh_shape {
                        let cell_min = self.get_cell_min_pos(USizeVec3::new(i, j, k));
                        let cell_aabb =
                            Aabb::new(cell_min, cell_min + Vec3A::splat(self.cell_size));

                        if !mesh_interface.check_overlap_with(&cell_aabb) {
                            continue;
                        }
                    }

                    for i1 in 0..=2 {
                        for j1 in 0..=2 {
                            for k1 in 0..=2 {
                                let mut cell = USizeVec3::new(i + i1, j + j1, k + k1);
                                if cell.cmpeq(USizeVec3::ZERO).any() {
                                    continue;
                                }

                                cell -= USizeVec3::ONE;

                                if cell.cmpge(self.num_cells).any() {
                                    continue;
                                }

                                let i = self.cell_indices_to_idx(cell);
                                if self.cells[i].static_handles.contains(&proxy_idx) {
                                    continue;
                                }

                                self.cells[i].static_handles.push(proxy_idx);
                            }
                        }
                    }
                }
            }
        }
    }
}

#[derive(Clone, Debug)]
pub struct GridBroadphase {
    cell_grid: CellGrid,
    handles: Vec<GridBroadphaseProxy>,
    pair_cache: HashedOverlappingPairCache,
}

impl GridBroadphase {
    pub fn new(min_pos: Vec3A, max_pos: Vec3A, cell_size: f32) -> Self {
        debug_assert!(min_pos.cmple(max_pos).all(), "Invalid min/max pos");

        let range = max_pos - min_pos;
        let num_cells = (range / cell_size)
            .ceil()
            .as_usizevec3()
            .max(USizeVec3::ONE);
        let total_cells = num_cells.element_product();
        let cells = repeat_with(GridCell::new).take(total_cells).collect();

        Self {
            cell_grid: CellGrid {
                max_pos,
                min_pos,
                cell_size,
                num_cells,
                cells,
            },
            handles: Vec::with_capacity(32),
            pair_cache: HashedOverlappingPairCache::default(),
        }
    }

    pub fn set_aabb(&mut self, aabb: Aabb) {
        self.handles[0].aabb = aabb;

        let new_indices = self.cell_grid.get_cell_indices(aabb.min);
        let new_idx = self.cell_grid.cell_indices_to_idx(new_indices);
        self.handles[0].cell_idx = new_idx;
    }

    pub fn create_proxy(&mut self, aabb: Aabb) -> usize {
        debug_assert!(aabb.min.cmple(aabb.max).all());

        let new_handle_idx = self.handles.len();
        let indices = self.cell_grid.get_cell_indices(aabb.min);
        let cell_idx = self.cell_grid.cell_indices_to_idx(indices);

        let new_handle = GridBroadphaseProxy {
            aabb,
            client_obj_idx: usize::MAX,
            cell_idx,
        };

        self.handles.push(new_handle);
        new_handle_idx
    }

    pub fn create_static_proxy(
        &mut self,
        aabb: Aabb,
        co: &RigidBody,
        world_array_idx: usize,
    ) -> usize {
        debug_assert!(aabb.min.cmple(aabb.max).all());

        let new_handle_idx = self.handles.len();
        let indices = self.cell_grid.get_cell_indices(aabb.min);
        let cell_idx = self.cell_grid.cell_indices_to_idx(indices);

        let new_handle = GridBroadphaseProxy {
            aabb,
            client_obj_idx: world_array_idx,
            cell_idx,
        };

        self.cell_grid
            .update_cells_static(&new_handle, co, new_handle_idx);

        self.handles.push(new_handle);
        new_handle_idx
    }

    pub fn calculate_overlapping_pairs(&mut self) {
        debug_assert!(self.pair_cache.is_empty());

        let proxy = &self.handles[0];
        let cell = &self.cell_grid.cells[proxy.cell_idx];
        for &other_proxy_idx in &cell.static_handles {
            let other_proxy = &self.handles[other_proxy_idx];

            if proxy.aabb.intersects(&other_proxy.aabb) {
                self.pair_cache.add_overlapping_pair(other_proxy_idx);
            }
        }
    }

    #[inline]
    pub fn process_all_overlapping_pairs(
        &mut self,
        ball_obj: &SphereRigidBody,
        collision_objs: &[RigidBody],
        dispatcher: &mut CollisionDispatcher,
        contact_added_callback: &mut ArenaContactTracker,
    ) {
        self.pair_cache.process_all_overlapping_pairs(
            ball_obj,
            collision_objs,
            dispatcher,
            &self.handles,
            contact_added_callback,
        );
    }
}
