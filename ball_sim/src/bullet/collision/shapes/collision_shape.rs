use std::sync::Arc;

use super::{bvh_triangle_mesh_shape::BvhTriangleMeshShape, static_plane_shape::StaticPlaneShape};
use crate::shared::Aabb;

#[derive(Clone)]
pub enum CollisionShapes {
    StaticPlane(StaticPlaneShape),
    TriangleMesh(Arc<BvhTriangleMeshShape>),
}

impl CollisionShapes {
    pub fn get_aabb(&self) -> Aabb {
        match self {
            Self::StaticPlane(shape) => shape.aabb_cache,
            Self::TriangleMesh(shape) => shape.aabb_ident_cache,
        }
    }
}
