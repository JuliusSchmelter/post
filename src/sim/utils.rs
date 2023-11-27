// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 22.11.23
// Last modified by Tibor Völcker on 27.11.23
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

pub const METER_PER_FOOT: f32 = 12. * 25.4 / 1000.;

// helper constant, as `f32::powi` is not a constant function
pub const SQUARE_METER_PER_SQUARE_FOOT: f32 = METER_PER_FOOT * METER_PER_FOOT;
pub const CUBIC_METER_PER_CUBIC_FOOT: f32 = METER_PER_FOOT * METER_PER_FOOT * METER_PER_FOOT;

pub const KELVIN_PER_RANKIN: f32 = 5. / 9.;
pub const KILOGRAM_PER_POUND: f32 = 4.5359237e-1;
pub const PASCAL_PER_PSF: f32 = KILOGRAM_PER_POUND * STD_GRAVITY / SQUARE_METER_PER_SQUARE_FOOT;

pub const STD_GRAVITY: f32 = 9.80665;
const AIR_MOLECULAR_WEIGHT: f32 = 28.9644;
const GAS_CONSTANT: f32 = 8.31432e3;
pub const AIR_KAPPA: f32 = 1.40;
pub const AIR_GAS_CONSTANT: f32 = GAS_CONSTANT / AIR_MOLECULAR_WEIGHT;
