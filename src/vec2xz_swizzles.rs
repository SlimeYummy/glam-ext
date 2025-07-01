use glam::{Vec3, Vec4, DVec3, DVec4};

use crate::{Vec2xz, DVec2xz};

macro_rules! vecxz_swizzles {
    ($type:ident, $vec3:ident, $vec4:ident) => {
        impl $type {
            #[inline]
            pub fn xx(self) -> Self {
                $type::splat(self.x)
            }

            #[inline]
            pub fn zx(self) -> Self {
                $type::new(self.z, self.x)
            }

            #[inline]
            pub fn zz(self) -> Self {
                $type::splat(self.z)
            }

            #[inline]
            pub fn xxx(self) -> $vec3 {
                $vec3::new(self.x, self.x, self.x)
            }

            #[inline]
            pub fn xxz(self) -> $vec3 {
                $vec3::new(self.x, self.x, self.z)
            }

            #[inline]
            pub fn xzx(self) -> $vec3 {
                $vec3::new(self.x, self.z, self.x)
            }

            #[inline]
            pub fn xzz(self) -> $vec3 {
                $vec3::new(self.x, self.z, self.z)
            }

            #[inline]
            pub fn zxx(self) -> $vec3 {
                $vec3::new(self.z, self.x, self.x)
            }

            #[inline]
            pub fn zxz(self) -> $vec3 {
                $vec3::new(self.z, self.x, self.z)
            }

            #[inline]
            pub fn zzx(self) -> $vec3 {
                $vec3::new(self.z, self.z, self.x)
            }

            #[inline]
            pub fn zzz(self) -> $vec3 {
                $vec3::new(self.z, self.z, self.z)
            }

            #[inline]
            pub fn xxxx(self) -> $vec4 {
                $vec4::new(self.x, self.x, self.x, self.x)
            }

            #[inline]
            pub fn xxxz(self) -> $vec4 {
                $vec4::new(self.x, self.x, self.x, self.z)
            }

            #[inline]
            pub fn xxzx(self) -> $vec4 {
                $vec4::new(self.x, self.x, self.z, self.x)
            }

            #[inline]
            pub fn xxzz(self) -> $vec4 {
                $vec4::new(self.x, self.x, self.z, self.z)
            }

            #[inline]
            pub fn xzxx(self) -> $vec4 {
                $vec4::new(self.x, self.z, self.x, self.x)
            }

            #[inline]
            pub fn xzxz(self) -> $vec4 {
                $vec4::new(self.x, self.z, self.x, self.z)
            }

            #[inline]
            pub fn xzzx(self) -> $vec4 {
                $vec4::new(self.x, self.z, self.z, self.x)
            }

            #[inline]
            pub fn xzzz(self) -> $vec4 {
                $vec4::new(self.x, self.z, self.z, self.z)
            }

            #[inline]
            pub fn zxxx(self) -> $vec4 {
                $vec4::new(self.z, self.x, self.x, self.x)
            }

            #[inline]
            pub fn zxxz(self) -> $vec4 {
                $vec4::new(self.z, self.x, self.x, self.z)
            }

            #[inline]
            pub fn zxzx(self) -> $vec4 {
                $vec4::new(self.z, self.x, self.z, self.x)
            }

            #[inline]
            pub fn zxzz(self) -> $vec4 {
                $vec4::new(self.z, self.x, self.z, self.z)
            }

            #[inline]
            pub fn zzxx(self) -> $vec4 {
                $vec4::new(self.z, self.z, self.x, self.x)
            }

            #[inline]
            pub fn zzxz(self) -> $vec4 {
                $vec4::new(self.z, self.z, self.x, self.z)
            }

            #[inline]
            pub fn zzzx(self) -> $vec4 {
                $vec4::new(self.z, self.z, self.z, self.x)
            }

            #[inline]
            pub fn zzzz(self) -> $vec4 {
                $vec4::new(self.z, self.z, self.z, self.z)
            }
        }        
    };
}

vecxz_swizzles!(Vec2xz, Vec3, Vec4);
vecxz_swizzles!(DVec2xz, DVec3, DVec4);
