use core::ops::Mul;
use glam::{Affine3A, Mat3, Mat4, Quat, Vec3, Vec3A};

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

        #[allow(clippy::useless_conversion)]
        let rotation = Quat::from_mat3(&Mat3::from_cols(
            (mat3.x_axis * inv_scale.x).into(),
            (mat3.y_axis * inv_scale.y).into(),
            (mat3.z_axis * inv_scale.z).into(),
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
    pub fn abs_diff_eq(&self, rhs: Self, max_abs_diff: f32) -> bool {
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
        let scale = 1.0 / self.scale;
        Self {
            translation: -self.rotation * (self.translation * scale),
            rotation: self.rotation.inverse(),
            scale,
        }
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
