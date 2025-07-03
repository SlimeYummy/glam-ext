use glam::{BVec2, Vec2, Vec3, Vec3A};
use core::fmt;
use core::iter::{Product, Sum};
use core::{f32, ops::*};

use super::math;

// A 2-dimensional vector in the xz plane of a 3D right-handed coordinate system.
// Most of the function definitions are the same as glam::Vec2.
// If there is no documentation, please refer to glam::Vec2 documentation.
#[derive(Clone, Copy, PartialEq)]
#[cfg_attr(feature = "bytemuck", derive(bytemuck::Pod, bytemuck::Zeroable))]
#[cfg_attr(feature = "cuda", repr(align(8)))]
#[repr(C)]
pub struct Vec2xz {
    pub x: f32,
    pub z: f32,
}

#[inline(always)]
const fn v2xz(v: Vec2) -> Vec2xz {
    Vec2xz { x: v.x, z: v.y }
}

#[inline(always)]
const fn xz2v(v: Vec2xz) -> Vec2 {
    Vec2::new(v.x, v.z)
}

impl From<Vec2> for Vec2xz {
    #[inline]
    fn from(v: Vec2) -> Self {
        v2xz(v)
    }
}

impl From<Vec2xz> for Vec2 {
    #[inline]
    fn from(v: Vec2xz) -> Self {
        xz2v(v)
    }
}

impl Vec2xz {
    pub const ZERO: Self = Self::splat(0.0);
    pub const ONE: Self = Self::splat(1.0);
    pub const NEG_ONE: Self = Self::splat(-1.0);
    pub const MIN: Self = Self::splat(f32::MIN);
    pub const MAX: Self = Self::splat(f32::MAX);
    pub const NAN: Self = Self::splat(f32::NAN);
    pub const INFINITY: Self = Self::splat(f32::INFINITY);
    pub const NEG_INFINITY: Self = Self::splat(f32::NEG_INFINITY);
    pub const X: Self = Self::new(1.0, 0.0);
    pub const Z: Self = Self::new(0.0, 1.0);
    pub const NEG_X: Self = Self::new(-1.0, 0.0);
    pub const NEG_Z: Self = Self::new(0.0, -1.0);
    pub const AXES: [Self; 2] = [Self::X, Self::Z];

    #[inline(always)]
    pub const fn new(x: f32, z: f32) -> Self {
        Self { x, z }
    }

    #[inline]
    pub const fn splat(v: f32) -> Self {
        Self { x: v, z: v }
    }

    #[inline]
    pub fn map<F>(self, f: F) -> Self
    where
        F: Fn(f32) -> f32,
    {
        Self::new(f(self.x), f(self.z))
    }

    #[inline]
    pub fn select(mask: BVec2, if_true: Self, if_false: Self) -> Self {
        v2xz(Vec2::select(mask, xz2v(if_true), xz2v(if_false)))
    }

    #[inline]
    pub const fn from_array(a: [f32; 2]) -> Self {
        Self::new(a[0], a[1])
    }

    #[inline]
    pub const fn to_array(&self) -> [f32; 2] {
        [self.x, self.z]
    }
    
    #[inline]
    pub const fn from_vec2(xy: Vec2) -> Self {
        v2xz(xy)
    }

    #[inline]
    pub const fn as_vec2(self) -> Vec2 {
        xz2v(self)
    }

    #[inline]
    pub const fn as_vec3(self) -> Vec3 {
        self.extend(0.0)
    }
    #[inline]
    pub const fn as_vec3a(self) -> Vec3A {
        Vec3A::new(self.x, 0.0, self.z)
    }

    #[inline]
    pub const fn from_slice(slice: &[f32]) -> Self {
        Vec2xz::new(slice[0], slice[1])
    }

    #[inline]
    pub fn write_to_slice(self, slice: &mut [f32]) {
        xz2v(self).write_to_slice(slice);
    }

    #[inline]
    pub const fn extend(self, y: f32) -> Vec3 {
        xz2v(self).extend(y)
    }

    #[inline]
    pub fn with_x(mut self, x: f32) -> Self {
        self.x = x;
        self
    }

    #[inline]
    pub fn with_z(mut self, z: f32) -> Self {
        self.z = z;
        self
    }

    #[inline]
    pub fn dot(self, rhs: Self) -> f32 {
        (self.x * rhs.x) + (self.z * rhs.z)
    }

    #[inline]
    pub fn dot_into_vec(self, rhs: Self) -> Self {
        Self::splat(self.dot(rhs))
    }

    #[inline]
    pub fn min(self, rhs: Self) -> Self {
        v2xz(Vec2::min(xz2v(self), xz2v(rhs)))
    }

    #[inline]
    pub fn max(self, rhs: Self) -> Self {
        v2xz(Vec2::max(xz2v(self), xz2v(rhs)))
    }

    #[inline]
    pub fn clamp(self, min: Self, max: Self) -> Self {
        v2xz(Vec2::clamp(xz2v(self), xz2v(min), xz2v(max)))
    }

    #[inline]
    pub fn min_element(self) -> f32 {
        xz2v(self).min_element()
    }

    #[inline]
    pub fn max_element(self) -> f32 {
        xz2v(self).max_element()
    }

    #[inline]
    pub fn element_sum(self) -> f32 {
        xz2v(self).element_sum()
    }

    #[inline]
    pub fn element_product(self) -> f32 {
        xz2v(self).element_product()
    }

    #[inline]
    pub fn cmpeq(self, rhs: Self) -> BVec2 {
        xz2v(self).cmpeq(xz2v(rhs))
    }

    #[inline]
    pub fn cmpne(self, rhs: Self) -> BVec2 {
        xz2v(self).cmpne(xz2v(rhs))
    }

    #[inline]
    pub fn cmpge(self, rhs: Self) -> BVec2 {
        xz2v(self).cmpge(xz2v(rhs))
    }

    #[inline]
    pub fn cmpgt(self, rhs: Self) -> BVec2 {
        xz2v(self).cmpgt(xz2v(rhs))
    }

    #[inline]
    pub fn cmple(self, rhs: Self) -> BVec2 {
        xz2v(self).cmple(xz2v(rhs))
    }

    #[inline]
    pub fn cmplt(self, rhs: Self) -> BVec2 {
        xz2v(self).cmplt(xz2v(rhs))
    }

    #[inline]
    pub fn abs(self) -> Self {
        v2xz(Vec2::abs(xz2v(self)))
    }

    #[inline]
    pub fn signum(self) -> Self {
        v2xz(Vec2::signum(xz2v(self)))
    }

    #[inline]
    pub fn copysign(self, rhs: Self) -> Self {
        v2xz(Vec2::copysign(xz2v(self), xz2v(rhs)))
    }

    #[inline]
    pub fn is_negative_bitmask(self) -> u32 {
        xz2v(self).is_negative_bitmask()
    }

    #[inline]
    pub fn is_finite(self) -> bool {
        xz2v(self).is_finite()
    }

    #[inline]
    pub fn is_finite_mask(self) -> BVec2 {
        xz2v(self).is_finite_mask()
    }

    #[inline]
    pub fn is_nan(self) -> bool {
        xz2v(self).is_nan()
    }

    #[inline]
    pub fn is_nan_mask(self) -> BVec2 {
        xz2v(self).is_nan_mask()
    }

    #[inline]
    pub fn length(self) -> f32 {
        xz2v(self).length()
    }

    #[inline]
    pub fn length_squared(self) -> f32 {
        xz2v(self).length_squared()
    }

    #[inline]
    pub fn length_recip(self) -> f32 {
        xz2v(self).length_recip()
    }

    #[inline]
    pub fn distance(self, rhs: Self) -> f32 {
        xz2v(self).distance(xz2v(rhs))
    }

    #[inline]
    pub fn distance_squared(self, rhs: Self) -> f32 {
        xz2v(self).distance_squared(xz2v(rhs))
    }

    #[inline]
    pub fn div_euclid(self, rhs: Self) -> Self {
        v2xz(xz2v(self).div_euclid(xz2v(rhs)))
    }

    #[inline]
    pub fn rem_euclid(self, rhs: Self) -> Self {
        v2xz(xz2v(self).rem_euclid(xz2v(rhs)))
    }

    #[inline]
    pub fn normalize(self) -> Self {
        v2xz(xz2v(self).normalize())
    }

    #[inline]
    pub fn try_normalize(self) -> Option<Self> {
        xz2v(self).try_normalize().map(v2xz)
    }

    #[inline]
    pub fn normalize_or(self, fallback: Self) -> Self {
        v2xz(xz2v(self).normalize_or(xz2v(fallback)))
    }

    #[inline]
    pub fn normalize_or_zero(self) -> Self {
        v2xz(xz2v(self).normalize_or_zero())
    }

    #[inline]
    pub fn is_normalized(self) -> bool {
        xz2v(self).is_normalized()
    }

    #[inline]
    pub fn project_onto(self, rhs: Self) -> Self {
        v2xz(xz2v(self).project_onto(xz2v(rhs)))
    }

    #[inline]
    pub fn reject_from(self, rhs: Self) -> Self {
        v2xz(xz2v(self).reject_from(xz2v(rhs)))
    }

    #[inline]
    pub fn project_onto_normalized(self, rhs: Self) -> Self {
        v2xz(xz2v(self).project_onto_normalized(xz2v(rhs)))
    }

    #[inline]
    pub fn reject_from_normalized(self, rhs: Self) -> Self {
        v2xz(xz2v(self).reject_from_normalized(xz2v(rhs)))
    }

    #[inline]
    pub fn round(self) -> Self {
        v2xz(xz2v(self).round())
    }

    #[inline]
    pub fn floor(self) -> Self {
        v2xz(xz2v(self).floor())
    }

    #[inline]
    pub fn ceil(self) -> Self {
        v2xz(xz2v(self).ceil())
    }

    #[inline]
    pub fn trunc(self) -> Self {
        v2xz(xz2v(self).trunc())
    }

    #[inline]
    pub fn fract(self) -> Self {
        v2xz(xz2v(self).fract())
    }

    #[inline]
    pub fn fract_gl(self) -> Self {
        v2xz(xz2v(self).fract_gl())
    }

    #[inline]
    pub fn exp(self) -> Self {
        v2xz(xz2v(self).exp())
    }

    #[inline]
    pub fn powf(self, n: f32) -> Self {
        v2xz(xz2v(self).powf(n))
    }

    #[inline]
    pub fn recip(self) -> Self {
        v2xz(xz2v(self).recip())
    }

    #[inline]
    pub fn lerp(self, rhs: Self, s: f32) -> Self {
        v2xz(xz2v(self).lerp(xz2v(rhs), s))
    }

    #[inline]
    pub fn move_towards(&self, rhs: Self, d: f32) -> Self {
        v2xz(xz2v(*self).move_towards(xz2v(rhs), d))
    }

    #[inline]
    pub fn midpoint(self, rhs: Self) -> Self {
        v2xz(xz2v(self).midpoint(xz2v(rhs)))
    }

    #[inline]
    pub fn abs_diff_eq(self, rhs: Self, max_abs_diff: f32) -> bool {
        xz2v(self).abs_diff_eq(xz2v(rhs), max_abs_diff)
    }

    #[inline]
    pub fn clamp_length(self, min: f32, max: f32) -> Self {
        v2xz(xz2v(self).clamp_length(min, max))
    }

    #[inline]
    pub fn clamp_length_max(self, max: f32) -> Self {
        v2xz(xz2v(self).clamp_length_max(max))
    }

    #[inline]
    pub fn clamp_length_min(self, min: f32) -> Self {
        v2xz(xz2v(self).clamp_length_min(min))
    }

    #[inline]
    pub fn mul_add(self, a: Self, b: Self) -> Self {
        v2xz(xz2v(self).mul_add(xz2v(a), xz2v(b)))
    }

    #[inline]
    pub fn reflect(self, normal: Self) -> Self {
        v2xz(xz2v(self).reflect(xz2v(normal)))
    }

    #[inline]
    pub fn refract(self, normal: Self, eta: f32) -> Self {
        v2xz(xz2v(self).refract(xz2v(normal), eta))
    }

    /// Creates a 2D vector containing `[angle.cos(), angle.sin()]`. This can be used in
    /// conjunction with the [`rotate()`][Self::rotate()] method, e.g.
    /// `Vec2xz::from_angle(PI).rotate(Vec2xz::Z)` will create the vector `[-1, 0]`
    /// and rotate [`Vec2xz::Z`] around it returning `-Vec2xz::Z`.
    #[inline]
    pub fn from_angle(angle: f32) -> Self {
        let (sin, cos) = math::sin_cos(angle);
        Self { x: cos, z: -sin }
    }

    /// Returns the angle (in radians) of this vector in the range `[-π, +π]`.
    ///
    /// The input does not need to be a unit vector however it must be non-zero.
    #[inline]
    pub fn to_angle(self) -> f32 {
        math::atan2(-self.z, self.x)
    }

    /// Returns the angle of rotation (in radians) from `self` to `rhs` in the range `[-π, +π]`.
    ///
    /// The inputs do not need to be unit vectors however they must be non-zero.
    #[inline]
    pub fn angle_to(self, rhs: Self) -> f32 {
        let angle = math::acos_approx(
            self.dot(rhs) / math::sqrt(self.length_squared() * rhs.length_squared()),
        );

        angle * math::signum(rhs.perp_dot(self))
    }

    /// Returns the angle of rotation (in radians) from `self` to `rhs` in the range `[-π, +π]`.
    ///
    /// Assumes the inputs are normalized and non-zero.
    #[inline]
    pub fn normalized_angle_to(self, rhs: Self) -> f32 {
        assert!(self.is_normalized());
        assert!(rhs.is_normalized());
        let angle = math::acos_approx(self.dot(rhs));
        angle * math::signum(rhs.perp_dot(self))
    }

    /// Return the sign of the angle between `self` and `rhs` in the range `[-1, +1]`.
    #[inline]
    pub fn angle_to_sign(self, rhs: Self) -> f32 {
        math::signum(rhs.perp_dot(self))
    }

    #[inline]
    pub fn perp(self) -> Self {
        v2xz(xz2v(self).perp())
    }

    #[inline]
    pub fn perp_dot(self, rhs: Self) -> f32 {
        xz2v(self).perp_dot(xz2v(rhs))
    }

    /// Returns `rhs` rotated by the angle of `self`. If `self` is normalized,
    /// then this just rotation. This is what you usually want. Otherwise,
    /// it will be like a rotation with a multiplication by `self`'s length.
    #[inline]
    pub fn rotate(self, rhs: Self) -> Self {
        Self {
            x: self.x * rhs.x - self.z * rhs.z,
            z: self.z * rhs.x + self.x * rhs.z,
        }
    }

    /// Rotates towards `rhs` up to `max_angle` (in radians).
    ///
    /// When `max_angle` is `0.0`, the result will be equal to `self`. When `max_angle` is equal to
    /// `self.angle_between(rhs)`, the result will be parallel to `rhs`. If `max_angle` is negative,
    /// rotates towards the exact opposite of `rhs`. Will not go past the target.
    #[inline]
    pub fn rotate_towards(&self, rhs: Self, max_angle: f32) -> Self {
        let a = self.angle_to(rhs);
        let abs_a = math::abs(a);
        // When `max_angle < 0`, rotate no further than `PI` radians away
        let angle = max_angle.clamp(abs_a - core::f32::consts::PI, abs_a) * math::signum(a);
        Self::from_angle(angle).rotate(*self)
    }

    #[inline]
    pub fn as_dvec2(&self) -> glam::DVec2 {
        xz2v(*self).as_dvec2()
    }

    #[inline]
    pub fn as_i8vec2(&self) -> glam::I8Vec2 {
        xz2v(*self).as_i8vec2()
    }

    #[inline]
    pub fn as_u8vec2(&self) -> glam::U8Vec2 {
        xz2v(*self).as_u8vec2()
    }

    #[inline]
    pub fn as_i16vec2(&self) -> glam::I16Vec2 {
        xz2v(*self).as_i16vec2()
    }

    #[inline]
    pub fn as_u16vec2(&self) -> glam::U16Vec2 {
        xz2v(*self).as_u16vec2()
    }

    #[inline]
    pub fn as_ivec2(&self) -> glam::IVec2 {
        xz2v(*self).as_ivec2()
    }

    #[inline]
    pub fn as_uvec2(&self) -> glam::UVec2 {
        xz2v(*self).as_uvec2()
    }

    #[inline]
    pub fn as_i64vec2(&self) -> glam::I64Vec2 {
        xz2v(*self).as_i64vec2()
    }

    #[inline]
    pub fn as_u64vec2(&self) -> glam::U64Vec2 {
        xz2v(*self).as_u64vec2()
    }

    // #[inline]
    // pub fn as_usizevec2(&self) -> glam::USizeVec2 {
    //     xz2v(*self).as_usizevec2()
    // }
}

impl Default for Vec2xz {
    #[inline(always)]
    fn default() -> Self {
        Self::ZERO
    }
}

macro_rules! impl_math {
    ($opt_trait:ident, $assign_trait:ident, $opt:ident, $assign:ident) => {
        impl $opt_trait for Vec2xz {
            type Output = Self;
            #[inline]
            fn $opt(self, rhs: Self) -> Self {
                Self {
                    x: self.x.$opt(rhs.x),
                    z: self.z.$opt(rhs.z),
                }
            }
        }

        impl $opt_trait<&Self> for Vec2xz {
            type Output = Self;
            #[inline]
            fn $opt(self, rhs: &Self) -> Self {
                self.$opt(*rhs)
            }
        }

        impl $opt_trait<&Vec2xz> for &Vec2xz {
            type Output = Vec2xz;
            #[inline]
            fn $opt(self, rhs: &Vec2xz) -> Vec2xz {
                (*self).$opt(*rhs)
            }
        }

        impl $opt_trait<Vec2xz> for &Vec2xz {
            type Output = Vec2xz;
            #[inline]
            fn $opt(self, rhs: Vec2xz) -> Vec2xz {
                (*self).$opt(rhs)
            }
        }

        impl $assign_trait for Vec2xz {
            #[inline]
            fn $assign(&mut self, rhs: Self) {
                self.x.$assign(rhs.x);
                self.z.$assign(rhs.z);
            }
        }

        impl $assign_trait<&Self> for Vec2xz {
            #[inline]
            fn $assign(&mut self, rhs: &Self) {
                self.$assign(*rhs);
            }
        }

        impl $opt_trait<f32> for Vec2xz {
            type Output = Self;
            #[inline]
            fn $opt(self, rhs: f32) -> Self {
                Self {
                    x: self.x.$opt(rhs),
                    z: self.z.$opt(rhs),
                }
            }
        }

        impl $opt_trait<&f32> for Vec2xz {
            type Output = Self;
            #[inline]
            fn $opt(self, rhs: &f32) -> Self {
                self.$opt(*rhs)
            }
        }

        impl $opt_trait<&f32> for &Vec2xz {
            type Output = Vec2xz;
            #[inline]
            fn $opt(self, rhs: &f32) -> Vec2xz {
                (*self).$opt(*rhs)
            }
        }

        impl $opt_trait<f32> for &Vec2xz {
            type Output = Vec2xz;
            #[inline]
            fn $opt(self, rhs: f32) -> Vec2xz {
                (*self).$opt(rhs)
            }
        }

        impl $assign_trait<f32> for Vec2xz {
            #[inline]
            fn $assign(&mut self, rhs: f32) {
                self.x.$assign(rhs);
                self.z.$assign(rhs);
            }
        }

        impl $assign_trait<&f32> for Vec2xz {
            #[inline]
            fn $assign(&mut self, rhs: &f32) {
                self.$assign(*rhs);
            }
        }

        impl $opt_trait<Vec2xz> for f32 {
            type Output = Vec2xz;
            #[inline]
            fn $opt(self, rhs: Vec2xz) -> Vec2xz {
                Vec2xz {
                    x: self.$opt(rhs.x),
                    z: self.$opt(rhs.z),
                }
            }
        }

        impl $opt_trait<&Vec2xz> for f32 {
            type Output = Vec2xz;
            #[inline]
            fn $opt(self, rhs: &Vec2xz) -> Vec2xz {
                self.$opt(*rhs)
            }
        }

        impl $opt_trait<&Vec2xz> for &f32 {
            type Output = Vec2xz;
            #[inline]
            fn $opt(self, rhs: &Vec2xz) -> Vec2xz {
                (*self).$opt(*rhs)
            }
        }

        impl $opt_trait<Vec2xz> for &f32 {
            type Output = Vec2xz;
            #[inline]
            fn $opt(self, rhs: Vec2xz) -> Vec2xz {
                (*self).$opt(rhs)
            }
        }
    };
}

impl_math!(Div, DivAssign, div, div_assign);
impl_math!(Mul, MulAssign, mul, mul_assign);
impl_math!(Add, AddAssign, add, add_assign);
impl_math!(Sub, SubAssign, sub, sub_assign);

impl AsRef<[f32; 2]> for Vec2xz {
    #[inline]
    fn as_ref(&self) -> &[f32; 2] {
        unsafe { &*(self as *const Self as *const [f32; 2]) }
    }
}

impl AsMut<[f32; 2]> for Vec2xz {
    #[inline]
    fn as_mut(&mut self) -> &mut [f32; 2] {
        unsafe { &mut *(self as *mut Self as *mut [f32; 2]) }
    }
}

impl Sum for Vec2xz {
    #[inline]
    fn sum<I>(iter: I) -> Self
    where
        I: Iterator<Item = Self>,
    {
        iter.fold(Self::ZERO, Self::add)
    }
}

impl<'a> Sum<&'a Self> for Vec2xz {
    #[inline]
    fn sum<I>(iter: I) -> Self
    where
        I: Iterator<Item = &'a Self>,
    {
        iter.fold(Self::ZERO, |a, &b| Self::add(a, b))
    }
}

impl Product for Vec2xz {
    #[inline]
    fn product<I>(iter: I) -> Self
    where
        I: Iterator<Item = Self>,
    {
        iter.fold(Self::ONE, Self::mul)
    }
}

impl<'a> Product<&'a Self> for Vec2xz {
    #[inline]
    fn product<I>(iter: I) -> Self
    where
        I: Iterator<Item = &'a Self>,
    {
        iter.fold(Self::ONE, |a, &b| Self::mul(a, b))
    }
}

impl Neg for Vec2xz {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self {
            x: self.x.neg(),
            z: self.z.neg(),
        }
    }
}

impl Neg for &Vec2xz {
    type Output = Vec2xz;
    #[inline]
    fn neg(self) -> Vec2xz {
        (*self).neg()
    }
}

impl Index<usize> for Vec2xz {
    type Output = f32;
    #[inline]
    fn index(&self, index: usize) -> &Self::Output {
        match index {
            0 => &self.x,
            1 => &self.z,
            _ => panic!("index out of bounds"),
        }
    }
}

impl IndexMut<usize> for Vec2xz {
    #[inline]
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match index {
            0 => &mut self.x,
            1 => &mut self.z,
            _ => panic!("index out of bounds"),
        }
    }
}

impl fmt::Display for Vec2xz {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if let Some(p) = f.precision() {
            write!(f, "[{:.*}, {:.*}]", p, self.x, p, self.z)
        } else {
            write!(f, "[{}, {}]", self.x, self.z)
        }
    }
}

impl fmt::Debug for Vec2xz {
    fn fmt(&self, fmt: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt.debug_tuple("Vec2xz")
            .field(&self.x)
            .field(&self.z)
            .finish()
    }
}

impl From<[f32; 2]> for Vec2xz {
    #[inline]
    fn from(a: [f32; 2]) -> Self {
        Self::new(a[0], a[1])
    }
}

impl From<Vec2xz> for [f32; 2] {
    #[inline]
    fn from(v: Vec2xz) -> Self {
        [v.x, v.z]
    }
}

impl From<(f32, f32)> for Vec2xz {
    #[inline]
    fn from(t: (f32, f32)) -> Self {
        Self::new(t.0, t.1)
    }
}

impl From<Vec2xz> for (f32, f32) {
    #[inline]
    fn from(v: Vec2xz) -> Self {
        (v.x, v.z)
    }
}

impl From<BVec2> for Vec2xz {
    #[inline]
    fn from(v: BVec2) -> Self {
        Self::new(f32::from(v.x), f32::from(v.y))
    }
}

#[cfg(feature = "approx")]
#[cfg(test)]
mod test {
    use std::f32::consts::{PI, FRAC_PI_2, FRAC_PI_3, FRAC_PI_4, FRAC_PI_6};
    use glam::{Quat, Vec3, Vec3Swizzles};
    use approx::assert_abs_diff_eq;

    use super::*;

    #[test]
    fn test_from_angle() {
        fn tester(angle: f32) {
            let r2 = Vec2xz::from_angle(angle);
            let r3 = Quat::from_rotation_y(angle);
            // println!("{:?} : {:?}", r2, Vec2::from_angle(angle));
            assert_abs_diff_eq!(r2.rotate(Vec2xz::X).as_vec2(), (r3 * Vec3::X).xz());
            assert_abs_diff_eq!(r2.rotate(Vec2xz::Z).as_vec2(), (r3 * Vec3::Z).xz());
        }

        tester(0.0);
        tester(FRAC_PI_6);
        tester(FRAC_PI_4);
        tester(FRAC_PI_2);
        tester(FRAC_PI_2 + FRAC_PI_6);
        tester(PI);
        tester(-FRAC_PI_6);
        tester(-FRAC_PI_4);
        tester(-FRAC_PI_2);
        tester(-FRAC_PI_2 - FRAC_PI_6);
        tester(-PI);
    }

    #[test]
    fn test_to_angle() {
        fn tester(angle: f32) {
            let v3 = Quat::from_rotation_y(angle) * Vec3::X;
            let a = Vec2xz::from_vec2(v3.xz()).to_angle();
            // println!("{:?} : {:?}", Vec2xz::from_vec2(v3.xz()).to_angle(), v3.xz().to_angle());
            assert_abs_diff_eq!(angle, a);
        }

        tester(0.0);
        tester(FRAC_PI_3);
        tester(FRAC_PI_4);
        tester(FRAC_PI_2);
        tester(FRAC_PI_2 + FRAC_PI_3);
        tester(-FRAC_PI_3);
        tester(-FRAC_PI_4);
        tester(-FRAC_PI_2);
        tester(-FRAC_PI_2 - FRAC_PI_3);

        let v3 = Quat::from_rotation_y(PI) * Vec3::X;
        let a = Vec2xz::from_vec2(v3.xz()).to_angle();
        assert_abs_diff_eq!(a, -PI, epsilon = 1e-6);

        let v3 = Quat::from_rotation_y(-PI) * Vec3::X;
        let a = Vec2xz::from_vec2(v3.xz()).to_angle();
        assert_abs_diff_eq!(a, PI, epsilon = 1e-6);
    }

    #[test]
    fn test_angle_to() {
        fn tester(angle_from: f32, angle_to: f32) {
            let v_from = Quat::from_rotation_y(angle_from) * Vec3::X;
            let v_to = Quat::from_rotation_y(angle_to) * Vec3::X;
            let a = Vec2xz::from_vec2(v_from.xz()).angle_to(Vec2xz::from_vec2(v_to.xz()));
            assert_abs_diff_eq!(angle_to - angle_from, a, epsilon = 1e-6);
        }

        tester(0.0, 0.0);
        tester(0.0, FRAC_PI_6);
        tester(0.0, FRAC_PI_4);
        tester(0.0, FRAC_PI_2);
        tester(0.0, FRAC_PI_2 + FRAC_PI_6);
        tester(0.0, -FRAC_PI_6);
        tester(0.0, -FRAC_PI_4);
        tester(0.0, -FRAC_PI_2);
        tester(0.0, -FRAC_PI_2 - FRAC_PI_6);

        tester(FRAC_PI_4, FRAC_PI_2);
        tester(-FRAC_PI_4, -FRAC_PI_2);
        tester(FRAC_PI_2, FRAC_PI_2 + FRAC_PI_6);
        tester(FRAC_PI_2, FRAC_PI_2 - FRAC_PI_6);
        
        let v_from = Quat::from_rotation_y(0.0) * Vec3::X;
        let v_to = Quat::from_rotation_y(PI) * Vec3::X;
        let a = Vec2xz::from_vec2(v_from.xz()).angle_to(Vec2xz::from_vec2(v_to.xz()));
        assert_abs_diff_eq!(-PI, a, epsilon = 1e-6);
    }

    #[test]
    fn test_rotate_towards() {
        assert_abs_diff_eq!(Vec2xz::X.rotate_towards(Vec2xz::Z, FRAC_PI_6), Vec2xz::from_angle(-FRAC_PI_6));
        assert_abs_diff_eq!(Vec2xz::Z.rotate_towards(Vec2xz::X, FRAC_PI_6), Vec2xz::from_angle(-FRAC_PI_3));
        
        assert_abs_diff_eq!(Vec2xz::X.rotate_towards(Vec2xz::Z, FRAC_PI_4), Vec2xz::from_angle(-FRAC_PI_4));
        assert_abs_diff_eq!(Vec2xz::X.rotate_towards(Vec2xz::Z, -FRAC_PI_4), Vec2xz::from_angle(FRAC_PI_4));
    }
}
