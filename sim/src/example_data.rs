// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 17.01.24
// Last modified by Tibor Völcker on 17.01.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::Vector3;

use utils::constants::*;
use utils::tables::linear_interpolation::Table2D;

use crate::vehicle::Engine;
use crate::Vehicle;

const DRAG_TABLE1: ([f64; 5], [f64; 12], [[f64; 12]; 5]) = (
    [-20., -5., 0., 5., 20.],
    [0., 0.5, 0.7, 0.8, 1., 1.2, 1.5, 2.0, 3., 5., 7., 10.],
    [
        [
            1.456, 1.585, 1.598, 1.242, 3.157, 2.996, 1.816, 1.301, 0.850, 0.482, 0.382, 0.396,
        ],
        [
            0.263, 0.338, 0.110, 0.302, 0.690, 0.671, 0.563, 0.480, 0.383, 0.256, 0.212, 0.210,
        ],
        [
            0.180, 0.18, 0.200, 0.251, 0.495, 0.502, 0.485, 0.456, 0.391, 0.272, 0.231, 0.231,
        ],
        [
            0.263, 0.338, 0.110, 0.302, 0.690, 0.671, 0.563, 0.480, 0.383, 0.256, 0.212, 0.210,
        ],
        [
            1.456, 1.585, 1.598, 1.242, 3.157, 2.996, 1.816, 1.301, 0.850, 0.482, 0.382, 0.396,
        ],
    ],
);

const LIFT_TABLE1: ([f64; 4], [f64; 12], [[f64; 12]; 4]) = (
    [-20., 0., 5., 20.],
    [0., 0.5, 0.7, 0.8, 1., 1.2, 1.5, 2.0, 3., 5., 7., 10.],
    [
        [
            -1.010, -1.025, -0.99, -0.815, -1.08, -1.11, -0.895, -0.788, -0.635, -0.480, -0.43,
            -0.43,
        ],
        [
            0.015, 0.04, 0.01, -0.045, 0.08, 0.038, -0.02, -0.108, -0.145, -0.15, -0.15, -0.15,
        ],
        [
            0.545, 0.75, 0.53, 0.365, 0.69, 0.638, 0.43, 0.242, 0.11, 0.025, 0.00, 0.00,
        ],
        [
            2.135, 2.24, 2.09, 1.595, 2.52, 2.438, 1.78, 1.292, 0.875, 0.55, 0.45, 0.45,
        ],
    ],
);

const DRAG_TABLE2: ([f64; 7], [f64; 12], [[f64; 12]; 7]) = (
    [-20., -4., 0., 5., 10., 20., 30.],
    [0., 0.2, 0.6, 0.8, 0.9, 1.3, 1.5, 2., 2.48, 3., 3.9, 40.],
    [
        [
            0.024, 0.024, 0.026, 0.028, 0.035, 0.93, 0.122, 0.116, 0.1, 0.092, 0.082, 0.03,
        ],
        [
            0.024, 0.024, 0.026, 0.028, 0.035, 0.93, 0.122, 0.116, 0.1, 0.092, 0.082, 0.03,
        ],
        [
            0.026, 0.026, 0.026, 0.024, 0.036, 0.092, 0.118, 0.106, 0.091, 0.082, 0.074, 0.022,
        ],
        [
            0.042, 0.042, 0.04, 0.042, 0.076, 0.124, 0.142, 0.124, 0.098, 0.088, 0.079, 0.033,
        ],
        [
            0.076, 0.076, 0.08, 0.1, 0.13, 0.194, 0.192, 0.165, 0.127, 0.114, 0.095, 0.057,
        ],
        [
            0.36, 0.36, 0.362, 0.44, 0.41, 0.39, 0.36, 0.32, 0.242, 0.224, 0.216, 0.238,
        ],
        [
            0.36, 0.36, 0.36, 0.44, 0.41, 0.39, 0.36, 0.32, 0.44, 0.418, 0.4, 0.3,
        ],
    ],
);

const LIFT_TABLE2: ([f64; 7], [f64; 12], [[f64; 12]; 7]) = (
    // The -4 is a 4 in the example. But the tests show, this must be a mistake
    // Otherways, the function does not even work
    [-20., -4., 0., 5., 10., 20., 30.],
    [0., 0.2, 0.6, 0.8, 0.9, 1.3, 1.5, 2., 2.48, 3., 3.9, 40.],
    [
        [
            -0.07, -0.08, -0.12, -0.12, -0.12, -0.12, -0.12, -0.13, -0.14, -0.12, -0.1, -0.14,
        ],
        [
            -0.07, -0.08, -0.12, -0.12, -0.12, -0.12, -0.12, -0.13, -0.14, -0.12, -0.1, -0.14,
        ],
        [
            // The last entry seems to be an input mistake in the example data
            0.08, 0.08, 0.08, 0.06, 0.06, 0.07, 0.04, 0.0, -0.02, -0.03, -0.04, 0.03,
        ],
        [
            0.29, 0.29, 0.29, 0.28, 0.28, 0.3, 0.24, 0.17, 0.12, 0.09, 0.08, 0.21,
        ],
        [
            0.5, 0.6, 0.49, 0.48, 0.52, 0.52, 0.41, 0.33, 0.25, 0.2, 0.15, 0.4,
        ],
        [
            0.94, 0.94, 0.92, 0.9, 0.94, 0.89, 0.75, 0.68, 0.67, 0.65, 0.62, 0.76,
        ],
        [
            0.94, 0.94, 0.92, 0.9, 0.94, 0.89, 0.75, 0.68, 0.67, 0.65, 0.62, 0.76,
        ],
    ],
);

pub struct DataPoint {
    pub time: f64,
    pub altitude: f64,
    pub temperature: f64,
    pub pressure: f64,
    pub density: f64,
    pub alpha: f64,
    pub mach: f64,
    pub cd: f64,
    pub cl: f64,
    pub throttle: f64,
    pub thrust: f64,
    pub dynamic_pressure: f64,
    pub aero_force: Vector3<f64>,
    pub vehicle: Vehicle,
}

pub fn example_data() -> [DataPoint; 4] {
    let vehicle1 = Vehicle::new(
        1.,
        4500. * SQUARE_METER_PER_SQUARE_FOOT,
        Table2D::new(DRAG_TABLE1.0, DRAG_TABLE1.1, DRAG_TABLE1.2),
        Table2D::new(LIFT_TABLE1.0, LIFT_TABLE1.1, LIFT_TABLE1.2),
        Table2D::zeros(),
        vec![Engine::new(
            [0., 0.],
            5472000.0 * NEWTON_PER_POUND_FORCE,
            232.5 * SQUARE_METER_PER_SQUARE_FOOT,
        )],
        [None, None, None],
    );
    let vehicle2 = Vehicle::new(
        1.,
        4840. * SQUARE_METER_PER_SQUARE_FOOT,
        Table2D::new(DRAG_TABLE2.0, DRAG_TABLE2.1, DRAG_TABLE2.2),
        Table2D::new(LIFT_TABLE2.0, LIFT_TABLE2.1, LIFT_TABLE2.2),
        Table2D::zeros(),
        vec![Engine::new(
            [0., 0.],
            1431000.0 * NEWTON_PER_POUND_FORCE,
            154.54 * SQUARE_METER_PER_SQUARE_FOOT,
        )],
        [None, None, None],
    );
    [
        DataPoint {
            time: 0.,
            altitude: -4.76837158e-7 * METER_PER_FOOT,
            temperature: 5.18670000e2 * KELVIN_PER_RANKIN,
            pressure: 2.11621660e3 * PASCAL_PER_PSF,
            density: 2.37690697e-3 * KILOGRAM_PER_SLUG / CUBIC_METER_PER_CUBIC_FOOT,
            alpha: 0.,
            mach: 0.,
            cd: 1.8e-1,
            cl: 1.5e-2,
            throttle: 1.,
            thrust: 4.97997964e6 * NEWTON_PER_POUND_FORCE,
            dynamic_pressure: 0. * PASCAL_PER_PSF,
            aero_force: Vector3::new(-0., 0., -0.) * NEWTON_PER_POUND_FORCE,
            vehicle: vehicle1.clone(),
        },
        DataPoint {
            time: 0.,
            altitude: 9.33310129e2 * METER_PER_FOOT,
            temperature: 5.15341815e2 * KELVIN_PER_RANKIN,
            pressure: 2.04581341e3 * PASCAL_PER_PSF,
            density: 2.31267089e-3 * KILOGRAM_PER_SLUG / CUBIC_METER_PER_CUBIC_FOOT,
            alpha: -1.48963081e-1,
            mach: 1.16179831e-1,
            cd: 1.82991982e-1,
            cl: 1.31054076e-2,
            throttle: 1.,
            thrust: 4.99634838e6 * NEWTON_PER_POUND_FORCE,
            dynamic_pressure: 1.93297185e1 * PASCAL_PER_PSF,
            aero_force: Vector3::new(-1.59202357e4, 0., -1.09857008e3) * NEWTON_PER_POUND_FORCE,
            vehicle: vehicle1,
        },
        DataPoint {
            time: 0.,
            altitude: 3.04960868e5 * METER_PER_FOOT,
            temperature: 3.41188380e2 * KELVIN_PER_RANKIN,
            pressure: 2.01763300e-3 * PASCAL_PER_PSF,
            density: 3.44501515e-9 * KILOGRAM_PER_SLUG / CUBIC_METER_PER_CUBIC_FOOT,
            alpha: 4.72845235e0,
            mach: 2.47287807e1,
            cd: 5.19996124e-2,
            cl: 1.46609433e-1,
            throttle: 7.50268212e-1,
            thrust: 1.07363350e6 * NEWTON_PER_POUND_FORCE,
            dynamic_pressure: 8.63665574e-1 * PASCAL_PER_PSF,
            aero_force: Vector3::new(-1.66106776e2, 0., -6.28680574e2) * NEWTON_PER_POUND_FORCE,
            vehicle: vehicle2.clone(),
        },
        DataPoint {
            time: 0.,
            altitude: 3.03804044e5 * METER_PER_FOOT,
            temperature: 3.39250012e2 * KELVIN_PER_RANKIN,
            pressure: 2.14880703e-3 * PASCAL_PER_PSF,
            density: 3.68995220e-9 * KILOGRAM_PER_SLUG / CUBIC_METER_PER_CUBIC_FOOT,
            alpha: 3.39554618e0,
            mach: 2.71263292e1,
            cd: 4.65609248e-2,
            cl: 1.12746181e-1,
            throttle: 6.50633622e-1,
            thrust: 9.31056380e5 * NEWTON_PER_POUND_FORCE,
            dynamic_pressure: 1.10682128e0 * PASCAL_PER_PSF,
            aero_force: Vector3::new(-2.13216514e2, 0., -6.17695942e2) * NEWTON_PER_POUND_FORCE,
            vehicle: vehicle2,
        },
    ]
}
