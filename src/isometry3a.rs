use core::ops::Mul;
use glam::{Affine3A, Mat3, Mat4, Quat, Vec3, Vec3A};

#[repr(C)]
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Isometry3A {
    pub translation: Vec3A,
    pub rotation: Quat,
}

impl Isometry3A {
    /// The degenerate zero isometry.
    ///
    /// This isometrys any finite vector and point to zero.
    /// The zero isometry is non-invertible.
    pub const ZERO: Self = Self {
        translation: Vec3A::ZERO,
        rotation: Quat::IDENTITY,
    };

    /// The identity isometry.
    ///
    /// Multiplying a vector with this returns the same vector.
    pub const IDENTITY: Self = Self {
        translation: Vec3A::ZERO,
        rotation: Quat::IDENTITY,
    };

    /// All NAN:s.
    pub const NAN: Self = Self {
        translation: Vec3A::NAN,
        rotation: Quat::NAN,
    };

    /// Creates a new isometry.
    #[inline]
    #[must_use]
    pub fn new(translation: Vec3, rotation: Quat) -> Self {
        Self {
            translation: translation.into(),
            rotation,
        }
    }

    /// Creates a isometry isometry from the given `rotation` quaternion.
    #[inline]
    #[must_use]
    pub fn from_quat(rotation: Quat) -> Self {
        Self {
            translation: Vec3A::ZERO,
            rotation,
        }
    }

    /// Creates a isometry isometry containing a 3D rotation around a normalized
    /// rotation `axis` of `angle` (in radians).
    #[inline]
    #[must_use]
    pub fn from_axis_angle(axis: Vec3, angle: f32) -> Self {
        Self {
            translation: Vec3A::ZERO,
            rotation: Quat::from_axis_angle(axis, angle),
        }
    }

    /// Creates a isometry isometry containing a 3D rotation around the x axis of
    /// `angle` (in radians).
    #[inline]
    #[must_use]
    pub fn from_rotation_x(angle: f32) -> Self {
        Self {
            translation: Vec3A::ZERO,
            rotation: Quat::from_rotation_x(angle),
        }
    }

    /// Creates a isometry isometry containing a 3D rotation around the y axis of
    /// `angle` (in radians).
    #[inline]
    #[must_use]
    pub fn from_rotation_y(angle: f32) -> Self {
        Self {
            translation: Vec3A::ZERO,
            rotation: Quat::from_rotation_y(angle),
        }
    }

    /// Creates a isometry isometry containing a 3D rotation around the z axis of
    /// `angle` (in radians).
    #[inline]
    #[must_use]
    pub fn from_rotation_z(angle: f32) -> Self {
        Self {
            translation: Vec3A::ZERO,
            rotation: Quat::from_rotation_z(angle),
        }
    }

    /// Creates a isometry isometryation from the given 3D `translation`.
    #[inline]
    #[must_use]
    pub fn from_translation(translation: Vec3) -> Self {
        Self {
            translation: translation.into(),
            rotation: Quat::IDENTITY,
        }
    }

    /// Creates a isometry from the given 3D `rotation` and `translation`.
    #[inline]
    #[must_use]
    pub fn from_rotation_translation(rotation: Quat, translation: Vec3) -> Self {
        Self {
            translation: translation.into(),
            rotation,
        }
    }

    /// Creates a isometry from a 3x3 matrix (expressing scale and rotation)
    #[inline]
    #[must_use]
    pub fn from_mat3(mat3: Mat3) -> Self {
        Self {
            translation: Vec3A::ZERO,
            rotation: Quat::from_mat3(&mat3),
        }
    }

    /// Creates a isometry from a 3x3 matrix (expressing scale and rotation)
    #[inline]
    #[must_use]
    pub fn from_mat3_translation(mat3: Mat3, translation: Vec3) -> Self {
        Self {
            translation: translation.into(),
            rotation: Quat::from_mat3(&mat3),
        }
    }

    /// Creates a isometry from a 4x4 matrix.
    #[inline]
    #[must_use]
    pub fn from_mat4(mat4: Mat4) -> Self {
        Self {
            translation: mat4.w_axis.truncate().into(),
            rotation: Quat::from_mat4(&mat4),
        }
    }

    /// Extracts `scale`, `rotation` and `translation` from `self`.
    #[inline]
    #[must_use]
    pub fn to_rotation_translation(&self) -> (Quat, Vec3) {
        (self.rotation, self.translation.into())
    }

    /// isometrys the given 3D points, applying scale, rotation and translation.
    #[inline]
    #[must_use]
    pub fn isometry_point3(&self, rhs: Vec3) -> Vec3 {
        let translation: Vec3 = self.translation.into();
        self.rotation * rhs + translation
    }

    /// isometrys the given 3D vector, applying scale and rotation (but NOT translation).
    #[inline]
    #[must_use]
    pub fn isometry_vector3(&self, rhs: Vec3) -> Vec3 {
        self.rotation * rhs
    }

    /// isometrys the given [`Vec3A`], applying scale, rotation and translation.
    #[inline]
    #[must_use]
    pub fn isometry_point3a(&self, rhs: Vec3A) -> Vec3A {
        self.rotation * rhs + self.translation
    }

    /// isometrys the given [`Vec3A`], applying scale and rotation (but NOT translation).
    #[inline]
    #[must_use]
    pub fn isometry_vector3a(&self, rhs: Vec3A) -> Vec3A {
        self.rotation * rhs
    }

    /// Returns `true` if, and only if, all elements are finite.
    ///
    /// If any element is either `NaN`, positive or negative infinity, this will return `false`.
    #[inline]
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.translation.is_finite() && self.rotation.is_finite()
    }

    /// Returns `true` if any elements are `NaN`.
    #[inline]
    #[must_use]
    pub fn is_nan(&self) -> bool {
        self.translation.is_nan() && self.rotation.is_nan()
    }

    /// Returns true if the absolute difference of all elements between `self` and `rhs`
    /// is less than or equal to `max_abs_diff`.
    #[inline]
    #[must_use]
    pub fn abs_diff_eq(&self, rhs: Self, max_abs_diff: f32) -> bool {
        self.translation.abs_diff_eq(rhs.translation, max_abs_diff)
            && self.rotation.abs_diff_eq(rhs.rotation, max_abs_diff)
    }

    /// Return the inverse of this isometry.
    ///
    /// Note that if the isometry is not invertible the result will be invalid.
    #[inline]
    #[must_use]
    pub fn inverse(&self) -> Self {
        Self {
            translation: -self.rotation * self.translation,
            rotation: self.rotation.inverse(),
        }
    }
}

impl From<Isometry3A> for Mat4 {
    #[inline]
    fn from(t: Isometry3A) -> Mat4 {
        let mat3 = Mat3::from_quat(t.rotation);
        Mat4::from_cols(
            mat3.x_axis.extend(0.0),
            mat3.y_axis.extend(0.0),
            mat3.z_axis.extend(0.0),
            t.translation.extend(1.0),
        )
    }
}

impl From<Isometry3A> for Affine3A {
    #[inline]
    fn from(t: Isometry3A) -> Affine3A {
        Affine3A::from_scale_rotation_translation(Vec3::ONE, t.rotation, t.translation.into())
    }
}

impl Mul<Mat4> for Isometry3A {
    type Output = Mat4;

    #[inline]
    fn mul(self, rhs: Mat4) -> Self::Output {
        Mat4::from(self) * rhs
    }
}

impl Mul<Isometry3A> for Mat4 {
    type Output = Mat4;

    #[inline]
    fn mul(self, rhs: Isometry3A) -> Self::Output {
        self * Mat4::from(rhs)
    }
}
