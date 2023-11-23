// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 22.11.23
// Last modified by Tibor Völcker on 23.11.23
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

pub const METER_PER_FOOT: f32 = 12. * 25.4 / 1000.;

// helper constant, as `f32::powi` is not a constant function
pub const SQUARE_METER_PER_SQUARE_FOOT: f32 = METER_PER_FOOT * METER_PER_FOOT;
pub const CUBIC_METER_PER_CUBIC_FOOT: f32 = METER_PER_FOOT * METER_PER_FOOT * METER_PER_FOOT;
