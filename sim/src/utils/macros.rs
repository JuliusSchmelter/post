// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 14.12.23
// Last modified by Tibor Völcker on 14.12.23
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

#[macro_export]
macro_rules! assert_lt {
    ($left: expr, $right: expr) => {
        assert!($left < $right, "assertion `left < right` failed\n  left: {}\n right: {}", $left, $right);
    };
    ($left: expr, $right: expr, $($arg:tt)+) => {
        assert!($left < $right, $($arg)*);
    };
}

#[macro_export]
macro_rules! assert_almost_eq {
    ($left: expr, $right: expr, $eps: expr) => {
        assert!(($left - $right as f64).abs() < $eps, "assertion `left ≈ right` failed\n    left: {}\n   right: {}\n epsilon: {}", $left, $right, $eps);
    };
    ($left: expr, $right: expr, $eps: expr, $($arg:tt)+) => {
        assert!(($left - $right as f64).abs() < $eps, $($arg)*);
    };
}

#[macro_export]
macro_rules! assert_almost_eq_rel {
    ($left: expr, $right: expr, $eps: expr) => {
        assert!((1. - ($left/$right)).abs() < $eps, "assertion `left ≈ right` failed\n    left: {}\n   right: {}\n epsilon: {}%", $left, $right, $eps * 100.);
    };
    ($left: expr, $right: expr, $eps: expr, $($arg:tt)+) => {
        assert!((1 - ($left/$right)).abs() < $eps, $($arg)*);
    };
}
