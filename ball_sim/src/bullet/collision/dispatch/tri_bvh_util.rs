use crate::{
    bullet::collision::shapes::{
        triangle_callback::ProcessTriangle, triangle_mesh::TriangleMesh,
        triangle_shape::TriangleShape,
    },
    shared::{Aabb, bvh::*},
};

pub struct NodeOverlapCallback<'a, T: ProcessTriangle> {
    tris: &'a [TriangleShape],
    aabbs: &'a [Aabb],
    callback: &'a mut T,
}

impl<'a, T: ProcessTriangle> NodeOverlapCallback<'a, T> {
    pub fn new(mesh_interface: &'a TriangleMesh, callback: &'a mut T) -> Self {
        let (tris, aabbs) = mesh_interface.get_tris_aabbs();

        Self {
            tris,
            aabbs,
            callback,
        }
    }
}

impl<T: ProcessTriangle> ProcessNode for NodeOverlapCallback<'_, T> {
    fn process_node(&mut self, node_triangle_idx: usize) {
        self.callback.process_triangle(
            &self.tris[node_triangle_idx],
            &self.aabbs[node_triangle_idx],
            node_triangle_idx,
        );
    }
}
