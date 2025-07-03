use rkyv::rancor::{Fallible};
use rkyv::{
    Archive, Deserialize, Serialize, Place
};

use crate::{Vec2xz, DVec2xz};

#[cfg(feature = "bytecheck")]
macro_rules! impl_rkyv {
    (@bytecheck $type:ty) => {
        // SAFETY: All bit patterns are valid for these primitive types.
        // https://docs.rs/bytecheck/0.8.1/src/bytecheck/lib.rs.html#352
        unsafe impl<C: Fallible +?Sized> rkyv::bytecheck::CheckBytes<C> for $type {
            #[inline]
            unsafe fn check_bytes(
                _value: *const Self,
                _: &mut C,
            ) -> Result<(), C::Error> {
                Ok(())
            }
        }
    };

    ($type:ty) => {
        impl_rkyv_derive!(@serialize $type);
        impl_rkyv_derive!(@archive_deserialize $type);
        impl_rkyv!(@bytecheck $type);
    };
}

#[cfg(not(feature = "bytecheck"))]
macro_rules! impl_rkyv {
    ($type:ty) => {
        impl_rkyv_derive!(@serialize $type);
        impl_rkyv_derive!(@archive_deserialize $type);
    };
}

macro_rules! impl_rkyv_derive {
    (@serialize $type:ty) => {
        impl<S: Fallible + ?Sized> Serialize<S> for $type {
            #[inline]
            fn serialize(&self, _: &mut S) -> Result<Self::Resolver, S::Error> {
                Ok(())
            }
        }
    };

    (@archive_deserialize $type:ty) => {
        // SAFETY: All glam types have a fully defined data layout.
        unsafe impl rkyv::traits::NoUndef for $type {}
        // SAFETY: All glam types have a stable, well-defined layout that is identical on all
        // targets.
        unsafe impl rkyv::Portable for $type {}
        impl Archive for $type {
            type Archived = $type;
            type Resolver = ();

            #[inline]
            fn resolve(&self, _: Self::Resolver, out: Place<Self::Archived>) {
                out.write(*self)
            }
        }

        impl<D: Fallible + ?Sized> Deserialize<$type, D> for $type {
            #[inline]
            fn deserialize(&self, _: &mut D) -> Result<$type, D::Error> {
                Ok(*self)
            }
        }
    };
}

impl_rkyv!(Vec2xz);
impl_rkyv!(DVec2xz);
