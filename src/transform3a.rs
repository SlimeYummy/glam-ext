use core::ops::Mul;
use glam::{Affine3A, Mat3, Mat3A, Mat4, Quat, Vec3, Vec3A};
use std::ops::MulAssign;

use crate::macros::glam_assert;

#[repr(C)]
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Transform3A {
    pub translation: Vec3A,
    pub rotation: Quat,
    pub scale: Vec3A,
}

impl Transform3A {
    /// The degenerate zero transform.
    ///
    /// This transforms any finite vector and point to zero.
    /// The zero transform is non-invertible.
    pub const ZERO: Self = Self {
        translation: Vec3A::ZERO,
        rotation: Quat::IDENTITY,
        scale: Vec3A::ONE,
    };

    /// The identity transform.
    ///
    /// Multiplying a vector with this returns the same vector.
    pub const IDENTITY: Self = Self {
        translation: Vec3A::ZERO,
        rotation: Quat::IDENTITY,
        scale: Vec3A::ONE,
    };

    /// All NAN:s.
    pub const NAN: Self = Self {
        translation: Vec3A::NAN,
        rotation: Quat::NAN,
        scale: Vec3A::NAN,
    };

    /// Creates a new transform.
    #[inline]
    #[must_use]
    pub fn new(translation: Vec3, rotation: Quat, scale: Vec3) -> Self {
        Self {
            translation: translation.into(),
            rotation,
            scale: scale.into(),
        }
    }

    /// Creates an affine transform that changes scale.
    /// Note that if any scale is zero the transform will be non-invertible.
    #[inline]
    #[must_use]
    pub fn from_scale(scale: Vec3) -> Self {
        Self {
            translation: Vec3A::ZERO,
            rotation: Quat::IDENTITY,
            scale: scale.into(),
        }
    }

    /// Creates a transform transform from the given `rotation` quaternion.
    #[inline]
    #[must_use]
    pub fn from_quat(rotation: Quat) -> Self {
        Self {
            translation: Vec3A::ZERO,
            rotation,
            scale: Vec3A::ONE,
        }
    }

    /// Creates a transform transform containing a 3D rotation around a normalized
    /// rotation `axis` of `angle` (in radians).
    #[inline]
    #[must_use]
    pub fn from_axis_angle(axis: Vec3, angle: f32) -> Self {
        Self {
            translation: Vec3A::ZERO,
            rotation: Quat::from_axis_angle(axis, angle),
            scale: Vec3A::ONE,
        }
    }

    /// Creates a transform transform containing a 3D rotation around the x axis of
    /// `angle` (in radians).
    #[inline]
    #[must_use]
    pub fn from_rotation_x(angle: f32) -> Self {
        Self {
            translation: Vec3A::ZERO,
            rotation: Quat::from_rotation_x(angle),
            scale: Vec3A::ONE,
        }
    }

    /// Creates a transform transform containing a 3D rotation around the y axis of
    /// `angle` (in radians).
    #[inline]
    #[must_use]
    pub fn from_rotation_y(angle: f32) -> Self {
        Self {
            translation: Vec3A::ZERO,
            rotation: Quat::from_rotation_y(angle),
            scale: Vec3A::ONE,
        }
    }

    /// Creates a transform transform containing a 3D rotation around the z axis of
    /// `angle` (in radians).
    #[inline]
    #[must_use]
    pub fn from_rotation_z(angle: f32) -> Self {
        Self {
            translation: Vec3A::ZERO,
            rotation: Quat::from_rotation_z(angle),
            scale: Vec3A::ONE,
        }
    }

    /// Creates a transform transformation from the given 3D `translation`.
    #[inline]
    #[must_use]
    pub fn from_translation(translation: Vec3) -> Self {
        Self {
            translation: translation.into(),
            rotation: Quat::IDENTITY,
            scale: Vec3A::ONE,
        }
    }

    /// Creates a transform from the given 3D `rotation` and `translation`.
    #[inline]
    #[must_use]
    pub fn from_rotation_translation(rotation: Quat, translation: Vec3) -> Self {
        Self {
            translation: translation.into(),
            rotation,
            scale: Vec3A::ONE,
        }
    }

    /// Creates a transform from the given 3D `scale`, `rotation` and `translation`.
    #[inline]
    #[must_use]
    pub fn from_scale_rotation_translation(scale: Vec3, rotation: Quat, translation: Vec3) -> Self {
        Self {
            translation: translation.into(),
            rotation,
            scale: scale.into(),
        }
    }

    /// Creates a transform from a 3x3 matrix (expressing scale and rotation)
    ///
    /// Note if the input matrix is non-uniform or shear, the result transform will be ill-defined.
    #[inline]
    #[must_use]
    pub fn from_mat3(mat3: Mat3) -> Self {
        Self::from_mat3_translation(mat3, Vec3::ZERO)
    }

    /// Creates a transform from a 3x3 matrix (expressing scale and rotation)
    ///
    /// Note if the input matrix is non-uniform or shear, the result transform will be ill-defined.
    #[inline]
    #[must_use]
    pub fn from_mat3_translation(mat3: Mat3, translation: Vec3) -> Self {
        use super::math;
        let det = mat3.determinant();
        glam_assert!(det != 0.0);

        let scale = Vec3::new(
            mat3.x_axis.length() * math::signum(det),
            mat3.y_axis.length(),
            mat3.z_axis.length(),
        );

        glam_assert!(scale.cmpne(Vec3::ZERO).all());

        let inv_scale = scale.recip();

        let rotation = Quat::from_mat3(&Mat3::from_cols(
            mat3.x_axis * inv_scale.x,
            mat3.y_axis * inv_scale.y,
            mat3.z_axis * inv_scale.z,
        ));
        Self {
            translation: translation.into(),
            rotation,
            scale: scale.into(),
        }
    }

    /// Creates a transform from a 4x4 matrix.
    ///
    /// Note if the input matrix is non-uniform or shear, the result transform will be ill-defined.
    #[inline]
    #[must_use]
    pub fn from_mat4(mat4: Mat4) -> Self {
        let translation = mat4.w_axis.truncate();
        Self::from_mat3_translation(Mat3::from_mat4(mat4), translation)
    }

    /// Extracts `scale`, `rotation` and `translation` from `self`.
    #[inline]
    #[must_use]
    pub fn to_scale_rotation_translation(&self) -> (Vec3, Quat, Vec3) {
        (self.scale.into(), self.rotation, self.translation.into())
    }

    /// Transforms the given 3D points, applying scale, rotation and translation.
    #[inline]
    #[must_use]
    pub fn transform_point3(&self, rhs: Vec3) -> Vec3 {
        let scale: Vec3 = self.scale.into();
        let translation: Vec3 = self.translation.into();
        self.rotation * (rhs * scale) + translation
    }

    /// Transforms the given 3D vector, applying scale and rotation (but NOT translation).
    #[inline]
    #[must_use]
    pub fn transform_vector3(&self, rhs: Vec3) -> Vec3 {
        let scale: Vec3 = self.scale.into();
        self.rotation * (rhs * scale)
    }

    /// Transforms the given [`Vec3A`], applying scale, rotation and translation.
    #[inline]
    #[must_use]
    pub fn transform_point3a(&self, rhs: Vec3A) -> Vec3A {
        self.rotation * (rhs * self.scale) + self.translation
    }

    /// Transforms the given [`Vec3A`], applying scale and rotation (but NOT translation).
    #[inline]
    #[must_use]
    pub fn transform_vector3a(&self, rhs: Vec3A) -> Vec3A {
        self.rotation * (rhs * self.scale)
    }

    /// Returns `true` if, and only if, all elements are finite.
    ///
    /// If any element is either `NaN`, positive or negative infinity, this will return `false`.
    #[inline]
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.translation.is_finite() && self.rotation.is_finite() && self.scale.is_finite()
    }

    /// Returns `true` if any elements are `NaN`.
    #[inline]
    #[must_use]
    pub fn is_nan(&self) -> bool {
        self.translation.is_nan() && self.rotation.is_nan() && self.scale.is_nan()
    }

    /// Returns true if the absolute difference of all elements between `self` and `rhs`
    /// is less than or equal to `max_abs_diff`.
    #[inline]
    #[must_use]
    pub fn abs_diff_eq(self, rhs: Self, max_abs_diff: f32) -> bool {
        self.translation.abs_diff_eq(rhs.translation, max_abs_diff)
            && self.rotation.abs_diff_eq(rhs.rotation, max_abs_diff)
            && self.scale.abs_diff_eq(rhs.scale, max_abs_diff)
    }

    /// Return the inverse of this transform.
    ///
    /// Note that if the transform is not invertible the result will be invalid.
    #[inline]
    #[must_use]
    pub fn inverse(&self) -> Self {
        let rot = Mat3A::from_quat(self.rotation);
        let mat_inv = Mat3A::from_cols(
            rot.x_axis * self.scale.x,
            rot.y_axis * self.scale.y,
            rot.z_axis * self.scale.z,
        )
        .inverse();
        let translation = -(mat_inv * self.translation);
        Transform3A::from_mat3_translation(mat_inv.into(), translation.into())
    }
}

impl From<Transform3A> for Mat4 {
    #[inline]
    fn from(t: Transform3A) -> Mat4 {
        let mat3 = Mat3::from_quat(t.rotation);
        Mat4::from_cols(
            (mat3.x_axis * t.scale.x).extend(0.0),
            (mat3.y_axis * t.scale.y).extend(0.0),
            (mat3.z_axis * t.scale.z).extend(0.0),
            t.translation.extend(1.0),
        )
    }
}

impl From<Transform3A> for Affine3A {
    #[inline]
    fn from(t: Transform3A) -> Affine3A {
        Affine3A::from_scale_rotation_translation(t.scale.into(), t.rotation, t.translation.into())
    }
}

impl Mul for Transform3A {
    type Output = Transform3A;

    #[inline]
    fn mul(self, rhs: Self) -> Self::Output {
        let rot1 = Mat3A::from_quat(self.rotation);
        let mat1 = Mat3A::from_cols(
            rot1.x_axis * self.scale.x,
            rot1.y_axis * self.scale.y,
            rot1.z_axis * self.scale.z,
        );
        let rot2 = Mat3A::from_quat(rhs.rotation);
        let mat2 = Mat3A::from_cols(
            rot2.x_axis * rhs.scale.x,
            rot2.y_axis * rhs.scale.y,
            rot2.z_axis * rhs.scale.z,
        );
        let translation = self.rotation * (self.scale * rhs.translation) + self.translation;
        Transform3A::from_mat3_translation((mat1 * mat2).into(), translation.into())
    }
}

impl MulAssign for Transform3A {
    #[inline]
    fn mul_assign(&mut self, rhs: Transform3A) {
        *self = self.mul(rhs);
    }
}

impl Mul<Mat4> for Transform3A {
    type Output = Mat4;

    #[inline]
    fn mul(self, rhs: Mat4) -> Self::Output {
        Mat4::from(self) * rhs
    }
}

impl Mul<Transform3A> for Mat4 {
    type Output = Mat4;

    #[inline]
    fn mul(self, rhs: Transform3A) -> Self::Output {
        self * Mat4::from(rhs)
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_from_mat4() {
        let scale = Vec3::new(0.5, 1.0, 2.0);
        let rot = Quat::from_rotation_y(-0.6);
        let pos = Vec3::new(1.0, -2.0, 3.0);
        let mat = Mat4::from_scale_rotation_translation(scale, rot, pos);
        let tran = Transform3A::from_mat4(mat);
        assert!(Vec3::abs_diff_eq(tran.scale.into(), scale, 1e-6));
        assert!(Quat::abs_diff_eq(tran.rotation, rot, 1e-6));
        assert!(Vec3::abs_diff_eq(tran.translation.into(), pos, 1e-6));
    }

    #[test]
    fn test_transform_point3() {
        let scale = Vec3::new(1.0, 0.7, 0.5);
        let rot = Quat::from_rotation_x(0.41);
        let pos = Vec3::new(1.1, 2.1, -3.1);
        let mat = Mat4::from_scale_rotation_translation(scale, rot, pos);
        let tran = Transform3A::from_mat4(mat);

        let point = Vec3::new(5.0, -5.0, 5.0);
        let p1 = mat.project_point3(point);
        let p2 = tran.transform_point3(point);
        assert!(Vec3::abs_diff_eq(p1, p2, 1e-6));

        let point = Vec3A::new(3.3, 4.4, 5.5);
        let p1 = mat.project_point3a(point);
        let p2 = tran.transform_point3a(point);
        assert!(Vec3A::abs_diff_eq(p1, p2, 1e-6));
    }

    #[test]
    fn test_transform_vec3() {
        let scale = Vec3::new(2.0, 2.0, 0.35);
        let rot = Quat::from_rotation_z(-0.2);
        let pos = Vec3::new(-1.5, 2.5, 4.5);
        let mat = Mat4::from_scale_rotation_translation(scale, rot, pos);
        let tran = Transform3A::from_mat4(mat);

        let vec = Vec3::new(1.0, 0.0, 0.7);
        let v1 = mat.transform_vector3(vec);
        let v2 = tran.transform_vector3(vec);
        assert!(Vec3::abs_diff_eq(v1, v2, 1e-6));

        let vec = Vec3A::new(-0.5, 1.0, 0.0);
        let v1 = mat.transform_vector3a(vec);
        let v2 = tran.transform_vector3a(vec);
        assert!(Vec3A::abs_diff_eq(v1, v2, 1e-6));
    }

    #[test]
    fn test_inverse() {
        let scale = Vec3::new(2.0, 1.7, 0.35);
        let rot = Quat::from_rotation_z(1.5) * Quat::from_rotation_x(1.0);
        let pos = Vec3::new(1.99, 0.77, -1.55);
        let mat = Mat4::from_scale_rotation_translation(scale, rot, pos);
        let mat_inv = mat.inverse();
        let tran1 = Transform3A::from_mat4(mat).inverse();
        let tran2 = Transform3A::from_mat4(mat_inv);
        assert!(Transform3A::abs_diff_eq(tran1, tran2, 1e-6));
    }

    #[test]
    fn test_mat4_from() {
        let scale = Vec3::new(3.1, 0.7, 1.11);
        let rot = Quat::from_rotation_y(-2.0);
        let pos = Vec3::new(3.0, 3.3, 3.33);
        let mat = Mat4::from_scale_rotation_translation(scale, rot, pos);
        let is = Transform3A::from_mat4(mat);
        let mat2 = Mat4::from(is);
        assert!(Mat4::abs_diff_eq(&mat, mat2, 1e-6));
    }

    #[test]
    fn test_transform_mul() {
        let scale1 = Vec3::new(1.1, 1.4, 1.7);
        let rot1 = Quat::from_rotation_x(0.77);
        let pos1 = Vec3::new(5.5, -6.6, 3.3);
        let mat1 = Mat4::from_scale_rotation_translation(scale1, rot1, pos1);
        let tran1 = Transform3A::from_scale_rotation_translation(scale1, rot1, pos1);

        let scale2 = Vec3::new(0.3, 1.0, 0.7);
        let rot2 = Quat::from_rotation_y(-0.44);
        let pos2 = Vec3::new(-4.4, -2.2, -3.3);
        let mat2 = Mat4::from_scale_rotation_translation(scale2, rot2, pos2);
        let tran2 = Transform3A::from_scale_rotation_translation(scale2, rot2, pos2);

        let mat = mat1 * mat2;
        let tran = tran1 * tran2;
        let tran_mat = Transform3A::from_mat4(mat);
        assert!(Transform3A::abs_diff_eq(tran, tran_mat, 1e-6));
    }
}
