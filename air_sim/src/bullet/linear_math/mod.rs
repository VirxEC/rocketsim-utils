use glam::{Mat3A, Quat, Vec3A, Vec4, Vec4Swizzles};
pub mod transform_util;

pub trait Mat3AExt {
    fn cofac(&self, r1: usize, c1: usize, r2: usize, c2: usize) -> f32;
    fn bullet_inverse(&self) -> Self;
    fn bullet_from_quat(q: Quat) -> Self;
}

impl Mat3AExt for Mat3A {
    fn cofac(&self, r1: usize, c1: usize, r2: usize, c2: usize) -> f32 {
        self.col(r1)[c1] * self.col(r2)[c2] - self.col(r1)[c2] * self.col(r2)[c1]
    }

    fn bullet_inverse(&self) -> Self {
        let co = Vec3A::new(
            self.cofac(1, 1, 2, 2),
            self.cofac(1, 2, 2, 0),
            self.cofac(1, 0, 2, 1),
        );
        let det = self.x_axis.dot(co);
        debug_assert_ne!(det, 0.0);
        let s = Vec3A::splat(det.recip());

        Self::from_cols(
            co * s,
            Vec3A::new(
                self.cofac(0, 2, 2, 1),
                self.cofac(0, 0, 2, 2),
                self.cofac(0, 1, 2, 0),
            ) * s,
            Vec3A::new(
                self.cofac(0, 1, 1, 2),
                self.cofac(0, 2, 1, 0),
                self.cofac(0, 0, 1, 1),
            ) * s,
        )
    }

    fn bullet_from_quat(q: Quat) -> Self {
        let d = q.length_squared();
        let s = 2.0 / d;

        let q: Vec4 = q.into();
        let nq = -q;

        let mut v1 = q.yxzw() * Vec4::new(-1.0, 1.0, 1.0, 1.0);
        let mut v2 = q.xxyw() * Vec4::new(-1.0, -1.0, 1.0, 1.0);
        let mut v3 = q.zyxw();

        let v11 = q.yyxw();
        let mut v21 = q.zzww();
        let v31 = q.xzxw() * Vec4::new(1.0, 1.0, -1.0, -1.0);

        v2 *= v1;
        v1 *= v11;
        v3 *= v31;

        let v11 = q.zwyw() * Vec4::new(-1.0, -1.0, 1.0, 1.0) * v21;
        v21.x *= -1.0;
        let mut v31 = q.wwyw() * Vec4::new(-1.0, 1.0, -1.0, -1.0);
        let y = nq.wzxw();
        let z = q.yxyw();

        v21 *= y;
        v31 *= z;

        v1 += v11;
        v2 += v21;
        v3 += v31;

        let vs = Vec4::new(s, s, s, 0.0);
        v1 *= vs;
        v2 *= vs;
        v3 *= vs;

        v1.x += 1.0;
        v2.y += 1.0;
        v3.z += 1.0;

        Self::from_cols(
            Vec3A::from_vec4(v1),
            Vec3A::from_vec4(v2),
            Vec3A::from_vec4(v3),
        )
        .transpose()
    }
}

pub trait QuatExt {
    fn bullet_mul_quat(self, q2: Self) -> Self;
    fn bullet_normalize(self) -> Self;
}

impl QuatExt for Quat {
    fn bullet_mul_quat(self, q2: Self) -> Self {
        const NEG_W: Vec4 = Vec4::new(1.0, 1.0, 1.0, -1.0);

        let q1: Vec4 = self.into();
        let q2: Vec4 = q2.into();

        let a2 = q1.yzxy() * q2.zxyy();
        let a1 = q1.xyzx() * q2.wwwx() + a2;

        let b1 = q1.zxyz() * q2.yzxz();
        let a0 = q1.wwww() * q2 - b1;

        let q = a0 + a1 * NEG_W;
        Self::from_vec4(q)
    }

    fn bullet_normalize(self) -> Self {
        let vec: Vec4 = self.into();
        let q = vec * vec.length().recip();
        Self::from_vec4(q)
    }
}
