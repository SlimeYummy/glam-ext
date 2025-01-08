use core::ops::{Mul, MulAssign};
use glam::{DAffine3, DMat3, DMat4, DQuat, DVec3};

#[repr(C)]
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct DIsometry3 {
    pub translation: DVec3,
    pub rotation: DQuat,
}

impl DIsometry3 {
    /// The degenerate zero isometry.
    ///
    /// This isometrys any finite vector and point to zero.
    /// The zero isometry is non-invertible.
    pub const ZERO: Self = Self {
        translation: DVec3::ZERO,
        rotation: DQuat::IDENTITY,
    };

    /// The identity isometry.
    ///
    /// Multiplying a vector with this returns the same vector.
    pub const IDENTITY: Self = Self {
        translation: DVec3::ZERO,
        rotation: DQuat::IDENTITY,
    };

    /// All NAN:s.
    pub const NAN: Self = Self {
        translation: DVec3::NAN,
        rotation: DQuat::NAN,
    };

    /// Creates a new isometry.
    #[inline]
    #[must_use]
    pub fn new(translation: DVec3, rotation: DQuat) -> Self {
        Self {
            translation,
            rotation,
        }
    }

    /// Creates a isometry isometry from the given `rotation` quaternion.
    #[inline]
    #[must_use]
    pub fn from_quat(rotation: DQuat) -> Self {
        Self {
            translation: DVec3::ZERO,
            rotation,
        }
    }

    /// Creates a isometry isometry containing a 3D rotation around a normalized
    /// rotation `axis` of `angle` (in radians).
    #[inline]
    #[must_use]
    pub fn from_axis_angle(axis: DVec3, angle: f64) -> Self {
        Self {
            translation: DVec3::ZERO,
            rotation: DQuat::from_axis_angle(axis, angle),
        }
    }

    /// Creates a isometry isometry containing a 3D rotation around the x axis of
    /// `angle` (in radians).
    #[inline]
    #[must_use]
    pub fn from_rotation_x(angle: f64) -> Self {
        Self {
            translation: DVec3::ZERO,
            rotation: DQuat::from_rotation_x(angle),
        }
    }

    /// Creates a isometry isometry containing a 3D rotation around the y axis of
    /// `angle` (in radians).
    #[inline]
    #[must_use]
    pub fn from_rotation_y(angle: f64) -> Self {
        Self {
            translation: DVec3::ZERO,
            rotation: DQuat::from_rotation_y(angle),
        }
    }

    /// Creates a isometry isometry containing a 3D rotation around the z axis of
    /// `angle` (in radians).
    #[inline]
    #[must_use]
    pub fn from_rotation_z(angle: f64) -> Self {
        Self {
            translation: DVec3::ZERO,
            rotation: DQuat::from_rotation_z(angle),
        }
    }

    /// Creates a isometry isometryation from the given 3D `translation`.
    #[inline]
    #[must_use]
    pub fn from_translation(translation: DVec3) -> Self {
        Self {
            translation,
            rotation: DQuat::IDENTITY,
        }
    }

    /// Creates a isometry from the given 3D `rotation` and `translation`.
    #[inline]
    #[must_use]
    pub fn from_rotation_translation(rotation: DQuat, translation: DVec3) -> Self {
        Self {
            translation,
            rotation,
        }
    }

    /// Creates a isometry from a 3x3 matrix (expressing scale and rotation)
    #[inline]
    #[must_use]
    pub fn from_mat3(mat3: DMat3) -> Self {
        Self {
            translation: DVec3::ZERO,
            rotation: DQuat::from_mat3(&mat3),
        }
    }

    /// Creates a isometry from a 3x3 matrix (expressing scale and rotation)
    #[inline]
    #[must_use]
    pub fn from_mat3_translation(mat3: DMat3, translation: DVec3) -> Self {
        Self {
            translation,
            rotation: DQuat::from_mat3(&mat3),
        }
    }

    /// Creates a isometry from a 4x4 matrix.
    #[inline]
    #[must_use]
    pub fn from_mat4(mat4: DMat4) -> Self {
        Self {
            translation: mat4.w_axis.truncate(),
            rotation: DQuat::from_mat4(&mat4),
        }
    }

    /// Extracts `scale`, `rotation` and `translation` from `self`.
    #[inline]
    #[must_use]
    pub fn to_rotation_translation(&self) -> (DQuat, DVec3) {
        (self.rotation, self.translation)
    }

    /// Transforms the given 3D points, applying rotation and translation.
    #[inline]
    #[must_use]
    pub fn transform_point3(&self, rhs: DVec3) -> DVec3 {
        let translation: DVec3 = self.translation;
        self.rotation * rhs + translation
    }

    /// Transforms the given 3D vector, applying rotation (but NOT translation).
    #[inline]
    #[must_use]
    pub fn transform_vector3(&self, rhs: DVec3) -> DVec3 {
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
    pub fn abs_diff_eq(self, rhs: Self, max_abs_diff: f64) -> bool {
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

impl From<DIsometry3> for DMat4 {
    #[inline]
    fn from(i: DIsometry3) -> DMat4 {
        let mat3 = DMat3::from_quat(i.rotation);
        DMat4::from_cols(
            mat3.x_axis.extend(0.0),
            mat3.y_axis.extend(0.0),
            mat3.z_axis.extend(0.0),
            i.translation.extend(1.0),
        )
    }
}

impl From<DIsometry3> for DAffine3 {
    #[inline]
    fn from(i: DIsometry3) -> DAffine3 {
        DAffine3::from_scale_rotation_translation(DVec3::ONE, i.rotation, i.translation)
    }
}

impl Mul for DIsometry3 {
    type Output = DIsometry3;

    #[inline]
    fn mul(self, rhs: DIsometry3) -> Self::Output {
        DIsometry3 {
            translation: self.rotation * rhs.translation + self.translation,
            rotation: self.rotation * rhs.rotation,
        }
    }
}

impl MulAssign for DIsometry3 {
    #[inline]
    fn mul_assign(&mut self, rhs: DIsometry3) {
        *self = self.mul(rhs);
    }
}

impl Mul<DMat4> for DIsometry3 {
    type Output = DMat4;

    #[inline]
    fn mul(self, rhs: DMat4) -> Self::Output {
        DMat4::from(self) * rhs
    }
}

impl Mul<DIsometry3> for DMat4 {
    type Output = DMat4;

    #[inline]
    fn mul(self, rhs: DIsometry3) -> Self::Output {
        self * DMat4::from(rhs)
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_from_mat4() {
        let rot = DQuat::from_rotation_y(0.6);
        let pos = DVec3::new(1.0, 2.0, 3.0);
        let mat = DMat4::from_rotation_translation(rot, pos);
        let is = DIsometry3::from_mat4(mat);
        assert!(DVec3::abs_diff_eq(is.translation, pos, 1e-6));
        assert!(DQuat::abs_diff_eq(is.rotation, rot, 1e-6));
    }

    #[test]
    fn test_transform_point3() {
        let rot = DQuat::from_rotation_x(0.6);
        let pos = DVec3::new(1.0, 2.0, 3.0);
        let mat = DMat4::from_rotation_translation(rot, pos);
        let is = DIsometry3::from_mat4(mat);

        let point = DVec3::new(5.0, -5.0, 5.0);
        let p1 = mat.project_point3(point);
        let p2 = is.transform_point3(point);
        assert!(DVec3::abs_diff_eq(p1, p2, 1e-6));
    }

    #[test]
    fn test_transform_vec3() {
        let rot = DQuat::from_rotation_z(-0.5);
        let pos = DVec3::new(1.5, -2.0, -2.0);
        let mat = DMat4::from_rotation_translation(rot, pos);
        let is = DIsometry3::from_mat4(mat);

        let vec = DVec3::new(1.0, 0.0, 0.7);
        let v1 = mat.transform_vector3(vec);
        let v2 = is.transform_vector3(vec);
        assert!(DVec3::abs_diff_eq(v1, v2, 1e-6));
    }

    #[test]
    fn test_inverse() {
        let rot = DQuat::from_rotation_z(1.5) * DQuat::from_rotation_x(1.0);
        let pos = DVec3::new(1.99, 0.77, -1.55);
        let mat = DMat4::from_rotation_translation(rot, pos);
        let mat_inv = mat.inverse();
        let is1 = DIsometry3::from_mat4(mat).inverse();
        let is2 = DIsometry3::from_mat4(mat_inv);
        assert!(DIsometry3::abs_diff_eq(is1, is2, 1e-6));
    }

    #[test]
    fn test_mat4_from() {
        let rot = DQuat::from_rotation_y(-2.0);
        let pos = DVec3::new(3.0, 3.3, 3.33);
        let mat = DMat4::from_rotation_translation(rot, pos);
        let is = DIsometry3::from_mat4(mat);
        let mat2 = DMat4::from(is);
        assert!(DMat4::abs_diff_eq(&mat, mat2, 1e-6));
    }

    #[test]
    fn test_isometry_mul() {
        let rot1 = DQuat::from_rotation_x(0.77);
        let pos1 = DVec3::new(6.6, -6.6, 3.3);
        let mat1 = DMat4::from_rotation_translation(rot1, pos1);
        let is1 = DIsometry3::from_rotation_translation(rot1, pos1);

        let rot2 = DQuat::from_rotation_z(-0.44);
        let pos2 = DVec3::new(-1.1, -2.2, -3.3);
        let mat2 = DMat4::from_rotation_translation(rot2, pos2);
        let is2 = DIsometry3::from_rotation_translation(rot2, pos2);

        let mat = mat1 * mat2;
        let is = is1 * is2;
        let is_mat = DIsometry3::from_mat4(mat);
        assert!(DIsometry3::abs_diff_eq(is, is_mat, 1e-6));
    }
}
