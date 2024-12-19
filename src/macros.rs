#[cfg(any(all(debug_assertions, feature = "debug-glam-assert"), feature = "glam-assert"))]
macro_rules! glam_assert {
    ($($arg:tt)*) => ( assert!($($arg)*); )
}
#[cfg(not(any(all(debug_assertions, feature = "debug-glam-assert"), feature = "glam-assert")))]
macro_rules! glam_assert {
    ($($arg:tt)*) => {};
}
pub(crate) use glam_assert;

// macro_rules! const_assert {
//     ($x:expr $(,)?) => {
//         #[allow(unknown_lints, clippy::eq_op)]
//         const _: () = assert!($x);
//     };
// }
// pub(crate) use const_assert;

// macro_rules! const_assert_eq {
//     ($x:expr, $y:expr $(,)?) => {
//         const_assert!($x == $y);
//     };
// }
// pub(crate) use const_assert_eq;
