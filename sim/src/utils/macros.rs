// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 17.01.24
// Last modified by Tibor Völcker on 22.05.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)

/// `assert_almost_eq_abs` is a macro used in the tests to assert if a float is
/// "nearly" equal to another value. "Nearly" equal means, if the value is
/// within an absolute margin.
///
/// # Examples
/// ```
/// # #[macro_use] extern crate sim;
/// # fn main() {
/// assert_almost_eq_abs!(1.35_f64, 1.3, 0.06);
/// # }
/// ```
/// ```should_panic
/// # #[macro_use] extern crate sim;
/// # fn main() {
/// assert_almost_eq_abs!(1.4_f64, 1.3, 0.04);
/// # }
/// ```
///
/// The macro can also compare vectors:
/// ```
/// # #[macro_use] extern crate sim;
/// # use nalgebra::vector;
/// # fn main() {
/// assert_almost_eq_abs!(vec vector![0.1_f64, 1.1, 2.1], vector![0., 1., 2.], 0.1);
/// # }
/// ```
///
/// __Note__: Because of floating arithmetic, this macro might fail if the
/// values are exactly equal.
#[macro_export]
macro_rules! assert_almost_eq_abs {
    (vec $left: expr, $right: expr, $eps: expr) => {
        // Iterate through vector
        for (i, _) in $left.iter().enumerate() {
            assert_almost_eq_abs!(
                $left[i],
                $right[i],
                $eps,
                format!("{}[{i}]", stringify!($left))
            );
        }
    };
    ($left: expr, $right: expr, $eps: expr) => {
        // Default display name is stringify value
        assert_almost_eq_abs!($left, $right, $eps, stringify!($left));
    };
    ($left: expr, $right: expr, $eps: expr, $disp: expr) => {
        // Implementation
        assert!(
            ($left - $right).abs() < $eps,
            "assertion `{}` ≈ {:.5} failed\n  `{}`: {:.5}\n  difference: {:.5} >= {}",
            $disp,
            $right,
            $disp,
            $left,
            ($left - $right).abs(),
            $eps
        );
    };
}

/// `assert_almost_eq_abs` is a macro used in the tests to assert if a float is
/// "nearly" equal to another value. "Nearly" equal means, if the value is
/// within an relative margin.
///
/// # Examples
/// ```
/// # #[macro_use] extern crate sim;
/// # fn main() {
/// assert_almost_eq_rel!(2.15_f64, 2.0_f64, 0.1);
/// # }
/// ```
/// ```should_panic
/// # #[macro_use] extern crate sim;
/// # fn main() {
/// assert_almost_eq_rel!(2.21_f64, 2.0_f64, 0.1);
/// # }
/// ```
///
/// If the right side is nearly zero, the relative error would skyrocket. That
/// is why the macro switches to use the [`assert_almost_eq_abs`] macro for
/// cases where the right side is < 1e-6.
/// ```
/// # #[macro_use] extern crate sim;
/// # fn main() {
/// assert_almost_eq_rel!(0.1_f64, 1e-7_f64, 0.002);
/// # }
/// ```
///
/// The macro can also compare vectors:
/// ```
/// # #[macro_use] extern crate sim;
/// # use nalgebra::vector;
/// # fn main() {
/// assert_almost_eq_rel!(vec vector![1.1_f64, 2.2, 3.3], vector![1_f64, 2., 3.], 0.11);
/// # }
/// ```
///
/// __Note__: Because of floating arithmetic, this macro might fail if the
/// values are exactly equal.
#[macro_export]
macro_rules! assert_almost_eq_rel {
    (vec $left: expr, $right: expr, $eps: expr) => {
        // Iterate through vector
        for (i, _) in $left.iter().enumerate() {
            assert_almost_eq_rel!(
                $left[i],
                $right[i],
                $eps,
                format!("{}[{i}]", stringify!($left))
            );
        }
    };
    ($left: expr, $right: expr, $eps: expr) => {
        // Default display name is stringify value
        assert_almost_eq_rel!($left, $right, $eps, stringify!($left));
    };
    ($left: expr, $right: expr, $eps: expr, $disp: expr) => {
        // Implementation
        if $right.abs() < 1e-6 {
            $crate::assert_almost_eq_abs!($left, $right, $eps * 100., $disp);
        } else {
            assert!(
                (1. - ($left / $right)).abs() < $eps,
                "assertion `{}` ≈ {:.5} failed\n  `{}`: {:.5}\n  difference: {:.3}% >= {}%",
                $disp,
                $right,
                $disp,
                $left,
                (1. - ($left / $right)).abs() * 100.,
                $eps * 100.
            );
        }
    };
}
