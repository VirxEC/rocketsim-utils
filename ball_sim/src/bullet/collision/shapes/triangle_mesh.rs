use glam::Vec3A;

use super::triangle_shape::TriangleShape;
use crate::{bullet::collision::shapes::triangle_callback::ProcessTriangle, shared::Aabb};

#[derive(Debug)]
pub struct TriangleMesh {
    triangles: Box<[TriangleShape]>,
    aabbs: Box<[Aabb]>,
}

impl TriangleMesh {
    pub fn new(verts: &[Vec3A], ids: &[usize]) -> Self {
        debug_assert_eq!(ids.len() % 3, 0);

        let triangles: Box<[TriangleShape]> = ids
            .as_chunks::<3>()
            .0
            .iter()
            .map(|ids| TriangleShape::from_points_iter(ids.iter().map(|&j| verts[j])))
            .collect();

        let aabbs = triangles.iter().map(|tri| tri.aabb()).collect();

        Self { triangles, aabbs }
    }

    pub fn internal_process_all_triangles<T: ProcessTriangle>(&self, callback: &mut T) {
        let (tris, aabbs) = self.get_tris_aabbs();

        for (i, (triangle, aabb)) in tris.iter().zip(aabbs).enumerate() {
            callback.process_triangle(triangle, aabb, i);
        }
    }

    pub fn get_triangle(&self, idx: usize) -> &TriangleShape {
        &self.triangles[idx]
    }

    pub fn get_total_num_faces(&self) -> usize {
        self.triangles.len()
    }

    pub fn get_tris_aabbs(&self) -> (&[TriangleShape], &[Aabb]) {
        (&self.triangles, &self.aabbs)
    }
}
