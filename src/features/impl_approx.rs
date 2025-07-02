use crate::{Vec2xz, DVec2xz};
use approx::{AbsDiffEq, RelativeEq, UlpsEq};

macro_rules! impl_approx_as_ref {
    ($prim:ident, $type:ty) => {
        impl AbsDiffEq for $type {
            type Epsilon = <$prim as AbsDiffEq>::Epsilon;
            fn default_epsilon() -> Self::Epsilon {
                $prim::default_epsilon()
            }
            fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
                self.as_ref().abs_diff_eq(other.as_ref(), epsilon)
            }
        }

        impl RelativeEq for $type {
            fn default_max_relative() -> Self::Epsilon {
                $prim::default_max_relative()
            }
            fn relative_eq(
                &self,
                other: &Self,
                epsilon: Self::Epsilon,
                max_relative: Self::Epsilon,
            ) -> bool {
                self.as_ref()
                    .relative_eq(other.as_ref(), epsilon, max_relative)
            }
        }

        impl UlpsEq for $type {
            fn default_max_ulps() -> u32 {
                $prim::default_max_ulps()
            }
            fn ulps_eq(&self, other: &Self, epsilon: Self::Epsilon, max_ulps: u32) -> bool {
                self.as_ref().ulps_eq(other.as_ref(), epsilon, max_ulps)
            }
        }
    };
}

impl_approx_as_ref!(f32, Vec2xz);
impl_approx_as_ref!(f64, DVec2xz);
