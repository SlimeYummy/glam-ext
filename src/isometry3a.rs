use core::ops::{Mul, MulAssign};
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

    /// Transforms the given 3D points, applying rotation and translation.
    #[inline]
    #[must_use]
    pub fn transform_point3(&self, rhs: Vec3) -> Vec3 {
        let translation: Vec3 = self.translation.into();
        self.rotation * rhs + translation
    }

    /// Transforms the given 3D vector, applying rotation (but NOT translation).
    #[inline]
    #[must_use]
    pub fn transform_vector3(&self, rhs: Vec3) -> Vec3 {
        self.rotation * rhs
    }

    /// Transforms the given [`Vec3A`], applying rotation and translation.
    #[inline]
    #[must_use]
    pub fn transform_point3a(&self, rhs: Vec3A) -> Vec3A {
        self.rotation * rhs + self.translation
    }

    /// Transforms the given [`Vec3A`], applying rotation (but NOT translation).
    #[inline]
    #[must_use]
    pub fn transform_vector3a(&self, rhs: Vec3A) -> Vec3A {
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
    pub fn abs_diff_eq(self, rhs: Self, max_abs_diff: f32) -> bool {
        self.translation.abs_diff_eq(rhs.translation, max_abs_diff)
            && self.rotation.abs_diff_eq(rhs.rotation, max_abs_diff)
    }

    /// Return the inverse of this isometry.
    ///
    /// Note that if the isometry is not invertible the result will be invalid.
    #[inline]
    #[must_use]
    pub fn inverse(&self) -> Self {
        let rotation = self.rotation.inverse();
        Self {
            translation: -(rotation * self.translation),
            rotation,
        }
    }
}

impl From<Isometry3A> for Mat4 {
    #[inline]
    fn from(i: Isometry3A) -> Mat4 {
        let mat3 = Mat3::from_quat(i.rotation);
        Mat4::from_cols(
            mat3.x_axis.extend(0.0),
            mat3.y_axis.extend(0.0),
            mat3.z_axis.extend(0.0),
            i.translation.extend(1.0),
        )
    }
}

impl From<Isometry3A> for Affine3A {
    #[inline]
    fn from(i: Isometry3A) -> Affine3A {
        Affine3A::from_scale_rotation_translation(Vec3::ONE, i.rotation, i.translation.into())
    }
}

impl Mul for Isometry3A {
    type Output = Isometry3A;

    #[inline]
    fn mul(self, rhs: Isometry3A) -> Self::Output {
        Isometry3A {
            translation: self.rotation * rhs.translation + self.translation,
            rotation: self.rotation * rhs.rotation,
        }
    }
}

impl MulAssign for Isometry3A {
    #[inline]
    fn mul_assign(&mut self, rhs: Isometry3A) {
        *self = self.mul(rhs);
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

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_from_mat4() {
        let rot = Quat::from_rotation_y(0.6);
        let pos = Vec3::new(1.0, 2.0, 3.0);
        let mat = Mat4::from_rotation_translation(rot, pos);
        let is = Isometry3A::from_mat4(mat);
        assert!(Vec3::abs_diff_eq(is.translation.into(), pos, 1e-6));
        assert!(Quat::abs_diff_eq(is.rotation, rot, 1e-6));
    }

    #[test]
    fn test_transform_point3() {
        let rot = Quat::from_rotation_x(0.6);
        let pos = Vec3::new(1.0, 2.0, 3.0);
        let mat = Mat4::from_rotation_translation(rot, pos);
        let is = Isometry3A::from_mat4(mat);

        let point = Vec3::new(5.0, -5.0, 5.0);
        let p1 = mat.project_point3(point);
        let p2 = is.transform_point3(point);
        assert!(Vec3::abs_diff_eq(p1, p2, 1e-6));

        let point = Vec3A::new(3.3, 4.4, 5.5);
        let p1 = mat.project_point3a(point);
        let p2 = is.transform_point3a(point);
        assert!(Vec3A::abs_diff_eq(p1, p2, 1e-6));
    }

    #[test]
    fn test_transform_vec3() {
        let rot = Quat::from_rotation_z(-0.5);
        let pos = Vec3::new(1.5, -2.0, -2.0);
        let mat = Mat4::from_rotation_translation(rot, pos);
        let is = Isometry3A::from_mat4(mat);

        let vec = Vec3::new(1.0, 0.0, 0.7);
        let v1 = mat.transform_vector3(vec);
        let v2 = is.transform_vector3(vec);
        assert!(Vec3::abs_diff_eq(v1, v2, 1e-6));

        let vec = Vec3A::new(-0.5, 1.0, 0.0);
        let v1 = mat.transform_vector3a(vec);
        let v2 = is.transform_vector3a(vec);
        assert!(Vec3A::abs_diff_eq(v1, v2, 1e-6));
    }

    #[test]
    fn test_inverse() {
        let rot = Quat::from_rotation_z(1.5) * Quat::from_rotation_x(1.0);
        let pos = Vec3::new(1.99, 0.77, -1.55);
        let mat = Mat4::from_rotation_translation(rot, pos);
        let mat_inv = mat.inverse();
        let is1 = Isometry3A::from_mat4(mat).inverse();
        let is2 = Isometry3A::from_mat4(mat_inv);
        assert!(Isometry3A::abs_diff_eq(is1, is2, 1e-6));
    }

    #[test]
    fn test_mat4_from() {
        let rot = Quat::from_rotation_y(-2.0);
        let pos = Vec3::new(3.0, 3.3, 3.33);
        let mat = Mat4::from_rotation_translation(rot, pos);
        let is = Isometry3A::from_mat4(mat);
        let mat2 = Mat4::from(is);
        assert!(Mat4::abs_diff_eq(&mat, mat2, 1e-6));
    }

    #[test]
    fn test_isometry_mul() {
        let rot1 = Quat::from_rotation_x(0.77);
        let pos1 = Vec3::new(6.6, -6.6, 3.3);
        let mat1 = Mat4::from_rotation_translation(rot1, pos1);
        let is1 = Isometry3A::from_rotation_translation(rot1, pos1);

        let rot2 = Quat::from_rotation_z(-0.44);
        let pos2 = Vec3::new(-1.1, -2.2, -3.3);
        let mat2 = Mat4::from_rotation_translation(rot2, pos2);
        let is2 = Isometry3A::from_rotation_translation(rot2, pos2);

        let mat = mat1 * mat2;
        let is = is1 * is2;
        let is_mat = Isometry3A::from_mat4(mat);
        assert!(Isometry3A::abs_diff_eq(is, is_mat, 1e-6));
    }
}
