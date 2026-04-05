use super::triangle_shape::TriangleShape;
use crate::shared::Aabb;

pub trait ProcessTriangle {
    fn process_triangle(&mut self, triangle: &TriangleShape, tri_aabb: &Aabb, triangle_idx: usize);
}
