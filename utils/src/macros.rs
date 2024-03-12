// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 17.01.24
// Last modified by Tibor Völcker on 12.03.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)
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
        if $right.abs() < $crate::NEARLY_ZERO {
            $crate::assert_almost_eq_abs!($left, $right, $eps, $disp);
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
