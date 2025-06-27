#[cfg(feature = "approx")]
use approx::{AbsDiffEq, RelativeEq, UlpsEq};
use core::ops::{Mul, MulAssign};
use glam::{DAffine3, DMat3, DMat4, DQuat, DVec3};

use crate::macros::glam_assert;

#[repr(C)]
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "rkyv", derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize))]
pub struct DTransform3 {
    pub translation: DVec3,
    pub rotation: DQuat,
    pub scale: DVec3,
}

impl DTransform3 {
    /// The degenerate zero transform.
    ///
    /// This transforms any finite vector and point to zero.
    /// The zero transform is non-invertible.
    pub const ZERO: Self = Self {
        translation: DVec3::ZERO,
        rotation: DQuat::IDENTITY,
        scale: DVec3::ONE,
    };

    /// The identity transform.
    ///
    /// Multiplying a vector with this returns the same vector.
    pub const IDENTITY: Self = Self {
        translation: DVec3::ZERO,
        rotation: DQuat::IDENTITY,
        scale: DVec3::ONE,
    };

    /// All NAN:s.
    pub const NAN: Self = Self {
        translation: DVec3::NAN,
        rotation: DQuat::NAN,
        scale: DVec3::NAN,
    };

    /// Creates a new transform.
    #[inline]
    #[must_use]
    pub fn new(translation: DVec3, rotation: DQuat, scale: DVec3) -> Self {
        Self {
            translation,
            rotation,
            scale,
        }
    }

    /// Creates an affine transform that changes scale.
    /// Note that if any scale is zero the transform will be non-invertible.
    #[inline]
    #[must_use]
    pub fn from_scale(scale: DVec3) -> Self {
        Self {
            translation: DVec3::ZERO,
            rotation: DQuat::IDENTITY,
            scale,
        }
    }

    /// Creates a transform transform from the given `rotation` quaternion.
    #[inline]
    #[must_use]
    pub fn from_quat(rotation: DQuat) -> Self {
        Self {
            translation: DVec3::ZERO,
            rotation,
            scale: DVec3::ONE,
        }
    }

    /// Creates a transform transform containing a 3D rotation around a normalized
    /// rotation `axis` of `angle` (in radians).
    #[inline]
    #[must_use]
    pub fn from_axis_angle(axis: DVec3, angle: f64) -> Self {
        Self {
            translation: DVec3::ZERO,
            rotation: DQuat::from_axis_angle(axis, angle),
            scale: DVec3::ONE,
        }
    }

    /// Creates a transform transform containing a 3D rotation around the x axis of
    /// `angle` (in radians).
    #[inline]
    #[must_use]
    pub fn from_rotation_x(angle: f64) -> Self {
        Self {
            translation: DVec3::ZERO,
            rotation: DQuat::from_rotation_x(angle),
            scale: DVec3::ONE,
        }
    }

    /// Creates a transform transform containing a 3D rotation around the y axis of
    /// `angle` (in radians).
    #[inline]
    #[must_use]
    pub fn from_rotation_y(angle: f64) -> Self {
        Self {
            translation: DVec3::ZERO,
            rotation: DQuat::from_rotation_y(angle),
            scale: DVec3::ONE,
        }
    }

    /// Creates a transform transform containing a 3D rotation around the z axis of
    /// `angle` (in radians).
    #[inline]
    #[must_use]
    pub fn from_rotation_z(angle: f64) -> Self {
        Self {
            translation: DVec3::ZERO,
            rotation: DQuat::from_rotation_z(angle),
            scale: DVec3::ONE,
        }
    }

    /// Creates a transform transformation from the given 3D `translation`.
    #[inline]
    #[must_use]
    pub fn from_translation(translation: DVec3) -> Self {
        Self {
            translation,
            rotation: DQuat::IDENTITY,
            scale: DVec3::ONE,
        }
    }

    /// Creates a transform from the given 3D `rotation` and `translation`.
    #[inline]
    #[must_use]
    pub fn from_rotation_translation(rotation: DQuat, translation: DVec3) -> Self {
        Self {
            translation,
            rotation,
            scale: DVec3::ONE,
        }
    }

    /// Creates a transform from the given 3D `scale`, `rotation` and `translation`.
    #[inline]
    #[must_use]
    pub fn from_scale_rotation_translation(scale: DVec3, rotation: DQuat, translation: DVec3) -> Self {
        Self {
            translation,
            rotation,
            scale,
        }
    }

    /// Creates a transform from a 3x3 matrix (expressing scale and rotation)
    ///
    /// Note if the input matrix is non-uniform or shear, the result transform will be ill-defined.
    #[inline]
    #[must_use]
    pub fn from_mat3(mat3: DMat3) -> Self {
        Self::from_mat3_translation(mat3, DVec3::ZERO)
    }

    /// Creates a transform from a 3x3 matrix (expressing scale and rotation)
    ///
    /// Note if the input matrix is non-uniform or shear, the result transform will be ill-defined.
    #[inline]
    #[must_use]
    pub fn from_mat3_translation(mat3: DMat3, translation: DVec3) -> Self {
        use super::math;
        let det = mat3.determinant();
        glam_assert!(det != 0.0);

        let scale = DVec3::new(
            mat3.x_axis.length() * math::signum(det),
            mat3.y_axis.length(),
            mat3.z_axis.length(),
        );

        glam_assert!(scale.cmpne(DVec3::ZERO).all());

        let inv_scale = scale.recip();

        let rotation = DQuat::from_mat3(&DMat3::from_cols(
            mat3.x_axis * inv_scale.x,
            mat3.y_axis * inv_scale.y,
            mat3.z_axis * inv_scale.z,
        ));
        Self {
            translation,
            rotation,
            scale,
        }
    }

    /// Creates a transform from a 4x4 matrix.
    ///
    /// Note if the input matrix is non-uniform or shear, the result transform will be ill-defined.
    #[inline]
    #[must_use]
    pub fn from_mat4(mat4: DMat4) -> Self {
        let translation = mat4.w_axis.truncate();
        Self::from_mat3_translation(DMat3::from_mat4(mat4), translation)
    }

    /// Extracts `scale`, `rotation` and `translation` from `self`.
    #[inline]
    #[must_use]
    pub fn to_scale_rotation_translation(&self) -> (DVec3, DQuat, DVec3) {
        (self.scale, self.rotation, self.translation)
    }

    /// Transforms the given 3D points, applying scale, rotation and translation.
    #[inline]
    #[must_use]
    pub fn transform_point3(&self, rhs: DVec3) -> DVec3 {
        let scale: DVec3 = self.scale;
        let translation: DVec3 = self.translation;
        self.rotation * (rhs * scale) + translation
    }

    /// Transforms the given 3D vector, applying scale and rotation (but NOT translation).
    #[inline]
    #[must_use]
    pub fn transform_vector3(&self, rhs: DVec3) -> DVec3 {
        let scale: DVec3 = self.scale;
        self.rotation * (rhs * scale)
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
    pub fn abs_diff_eq(self, rhs: Self, max_abs_diff: f64) -> bool {
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
        let rot = DMat3::from_quat(self.rotation);
        let mat_inv = DMat3::from_cols(
            rot.x_axis * self.scale.x,
            rot.y_axis * self.scale.y,
            rot.z_axis * self.scale.z,
        )
        .inverse();
        let translation = -(mat_inv * self.translation);
        DTransform3::from_mat3_translation(mat_inv, translation)
    }
}

impl From<DTransform3> for DMat4 {
    #[inline]
    fn from(t: DTransform3) -> DMat4 {
        let mat3 = DMat3::from_quat(t.rotation);
        DMat4::from_cols(
            (mat3.x_axis * t.scale.x).extend(0.0),
            (mat3.y_axis * t.scale.y).extend(0.0),
            (mat3.z_axis * t.scale.z).extend(0.0),
            t.translation.extend(1.0),
        )
    }
}

impl From<DTransform3> for DAffine3 {
    #[inline]
    fn from(t: DTransform3) -> DAffine3 {
        DAffine3::from_scale_rotation_translation(t.scale, t.rotation, t.translation)
    }
}

impl Mul for DTransform3 {
    type Output = DTransform3;

    #[inline]
    fn mul(self, rhs: Self) -> Self::Output {
        let rot1 = DMat3::from_quat(self.rotation);
        let mat1 = DMat3::from_cols(
            rot1.x_axis * self.scale.x,
            rot1.y_axis * self.scale.y,
            rot1.z_axis * self.scale.z,
        );
        let rot2 = DMat3::from_quat(rhs.rotation);
        let mat2 = DMat3::from_cols(
            rot2.x_axis * rhs.scale.x,
            rot2.y_axis * rhs.scale.y,
            rot2.z_axis * rhs.scale.z,
        );
        let translation = self.rotation * (self.scale * rhs.translation) + self.translation;
        DTransform3::from_mat3_translation(mat1 * mat2, translation)
    }
}

impl MulAssign for DTransform3 {
    #[inline]
    fn mul_assign(&mut self, rhs: DTransform3) {
        *self = self.mul(rhs);
    }
}

impl Mul<DMat4> for DTransform3 {
    type Output = DMat4;

    #[inline]
    fn mul(self, rhs: DMat4) -> Self::Output {
        DMat4::from(self) * rhs
    }
}

impl Mul<DTransform3> for DMat4 {
    type Output = DMat4;

    #[inline]
    fn mul(self, rhs: DTransform3) -> Self::Output {
        self * DMat4::from(rhs)
    }
}

#[cfg(feature = "approx")]
impl AbsDiffEq for DTransform3 {
    type Epsilon = <f64 as AbsDiffEq>::Epsilon;

    #[inline]
    fn default_epsilon() -> Self::Epsilon {
        f64::default_epsilon()
    }

    #[inline]
    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        self.translation.abs_diff_eq(other.translation, epsilon)
            && self.rotation.abs_diff_eq(other.rotation, epsilon)
            && self.scale.abs_diff_eq(other.scale, epsilon)
    }
}

#[cfg(feature = "approx")]
impl RelativeEq for DTransform3 {
    #[inline]
    fn default_max_relative() -> Self::Epsilon {
        f64::default_max_relative()
    }

    #[inline]
    fn relative_eq(&self, other: &Self, epsilon: Self::Epsilon, max_relative: Self::Epsilon) -> bool {
        self.translation.relative_eq(&other.translation, epsilon, max_relative)
            && self.rotation.relative_eq(&other.rotation, epsilon, max_relative)
            && self.scale.relative_eq(&other.scale, epsilon, max_relative)
    }
}

#[cfg(feature = "approx")]
impl UlpsEq for DTransform3 {
    #[inline]
    fn default_max_ulps() -> u32 {
        f64::default_max_ulps()
    }

    #[inline]
    fn ulps_eq(&self, other: &Self, epsilon: Self::Epsilon, max_ulps: u32) -> bool {
        self.translation.ulps_eq(&other.translation, epsilon, max_ulps)
            && self.rotation.ulps_eq(&other.rotation, epsilon, max_ulps)
            && self.scale.ulps_eq(&other.scale, epsilon, max_ulps)
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_from_mat4() {
        let scale = DVec3::new(0.5, 1.0, 2.0);
        let rot = DQuat::from_rotation_y(-0.6);
        let pos = DVec3::new(1.0, -2.0, 3.0);
        let mat = DMat4::from_scale_rotation_translation(scale, rot, pos);
        let tran = DTransform3::from_mat4(mat);
        assert!(DVec3::abs_diff_eq(tran.scale, scale, 1e-6));
        assert!(DQuat::abs_diff_eq(tran.rotation, rot, 1e-6));
        assert!(DVec3::abs_diff_eq(tran.translation, pos, 1e-6));
    }

    #[test]
    fn test_transform_point3() {
        let scale = DVec3::new(1.0, 0.7, 0.5);
        let rot = DQuat::from_rotation_x(0.41);
        let pos = DVec3::new(1.1, 2.1, -3.1);
        let mat = DMat4::from_scale_rotation_translation(scale, rot, pos);
        let tran = DTransform3::from_mat4(mat);

        let point = DVec3::new(5.0, -5.0, 5.0);
        let p1 = mat.project_point3(point);
        let p2 = tran.transform_point3(point);
        assert!(DVec3::abs_diff_eq(p1, p2, 1e-6));
    }

    #[test]
    fn test_transform_vec3() {
        let scale = DVec3::new(2.0, 2.0, 0.35);
        let rot = DQuat::from_rotation_z(-0.2);
        let pos = DVec3::new(-1.5, 2.5, 4.5);
        let mat = DMat4::from_scale_rotation_translation(scale, rot, pos);
        let tran = DTransform3::from_mat4(mat);

        let vec = DVec3::new(1.0, 0.0, 0.7);
        let v1 = mat.transform_vector3(vec);
        let v2 = tran.transform_vector3(vec);
        assert!(DVec3::abs_diff_eq(v1, v2, 1e-6));
    }

    #[test]
    fn test_inverse() {
        let scale = DVec3::new(2.0, 1.7, 0.35);
        let rot = DQuat::from_rotation_z(1.5) * DQuat::from_rotation_x(1.0);
        let pos = DVec3::new(1.99, 0.77, -1.55);
        let mat = DMat4::from_scale_rotation_translation(scale, rot, pos);
        let mat_inv = mat.inverse();
        let tran1 = DTransform3::from_mat4(mat).inverse();
        let tran2 = DTransform3::from_mat4(mat_inv);
        assert!(DTransform3::abs_diff_eq(tran1, tran2, 1e-6));
    }

    #[test]
    fn test_mat4_from() {
        let scale = DVec3::new(3.1, 0.7, 1.11);
        let rot = DQuat::from_rotation_y(-2.0);
        let pos = DVec3::new(3.0, 3.3, 3.33);
        let mat = DMat4::from_scale_rotation_translation(scale, rot, pos);
        let is = DTransform3::from_mat4(mat);
        let mat2 = DMat4::from(is);
        assert!(DMat4::abs_diff_eq(&mat, mat2, 1e-6));
    }

    #[test]
    fn test_transform_mul() {
        let scale1 = DVec3::new(1.1, 1.4, 1.7);
        let rot1 = DQuat::from_rotation_x(0.77);
        let pos1 = DVec3::new(5.5, -6.6, 3.3);
        let mat1 = DMat4::from_scale_rotation_translation(scale1, rot1, pos1);
        let tran1 = DTransform3::from_scale_rotation_translation(scale1, rot1, pos1);

        let scale2 = DVec3::new(0.3, 1.0, 0.7);
        let rot2 = DQuat::from_rotation_y(-0.44);
        let pos2 = DVec3::new(-4.4, -2.2, -3.3);
        let mat2 = DMat4::from_scale_rotation_translation(scale2, rot2, pos2);
        let tran2 = DTransform3::from_scale_rotation_translation(scale2, rot2, pos2);

        let mat = mat1 * mat2;
        let tran = tran1 * tran2;
        let tran_mat = DTransform3::from_mat4(mat);
        assert!(DTransform3::abs_diff_eq(tran, tran_mat, 1e-6));
    }
}
