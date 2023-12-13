// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 22.11.23
// Last modified by Tibor Völcker on 09.12.23
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

pub const METER_PER_FOOT: f64 = 12. * 25.4 / 1000.;

// helper constant, as `f64::powi` is not a constant function
pub const SQUARE_METER_PER_SQUARE_FOOT: f64 = METER_PER_FOOT * METER_PER_FOOT;
pub const CUBIC_METER_PER_CUBIC_FOOT: f64 = METER_PER_FOOT * METER_PER_FOOT * METER_PER_FOOT;

pub const KELVIN_PER_RANKIN: f64 = 5. / 9.;
pub const KILOGRAM_PER_POUND: f64 = 4.5359237e-1;
pub const PASCAL_PER_PSF: f64 = KILOGRAM_PER_POUND * STD_GRAVITY / SQUARE_METER_PER_SQUARE_FOOT;

pub const STD_GRAVITY: f64 = 9.80665;
const AIR_MOLECULAR_WEIGHT: f64 = 28.9644;
const GAS_CONSTANT: f64 = 8.31432e3;
pub const AIR_KAPPA: f64 = 1.40;
pub const AIR_GAS_CONSTANT: f64 = GAS_CONSTANT / AIR_MOLECULAR_WEIGHT;

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
