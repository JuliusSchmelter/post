// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 17.01.24
// Last modified by Tibor Völcker on 21.03.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)
#![cfg(test)]
pub use data::DATA_POINTS;
pub use steering::PITCH_RATES;
pub use vehicles::VEHICLES;

mod vehicles {
    use crate::constants::*;
    use crate::state::StateVariable;
    use crate::utils::Table2D;
    use crate::vehicle::Engine;
    use crate::Vehicle;
    use lazy_static::lazy_static;

    lazy_static! {
        static ref DRAG_TABLE1: ([f64; 5], [f64; 12], [[f64; 12]; 5]) = (
            [
                -20_f64.to_radians(),
                -5_f64.to_radians(),
                0.,
                5_f64.to_radians(),
                20_f64.to_radians(),
            ],
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

        static ref LIFT_TABLE1: ([f64; 4], [f64; 12], [[f64; 12]; 4]) = (
            [
                -20_f64.to_radians(),
                0.,
                5_f64.to_radians(),
                20_f64.to_radians(),
            ],
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

        static ref DRAG_TABLE2: ([f64; 7], [f64; 12], [[f64; 12]; 7]) = (
            [
                -20_f64.to_radians(),
                -4_f64.to_radians(),
                0.,
                5_f64.to_radians(),
                10_f64.to_radians(),
                20_f64.to_radians(),
                30_f64.to_radians(),
            ],
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

        static ref LIFT_TABLE2: ([f64; 7], [f64; 12], [[f64; 12]; 7]) = (
            // The -4 is a 4 in the example. But the tests show, this must be a mistake
            // Otherways, the function does not even work
            [
                -20_f64.to_radians(),
                -4_f64.to_radians(),
                0.,
                5_f64.to_radians(),
                10_f64.to_radians(),
                20_f64.to_radians(),
                30_f64.to_radians(),
            ],
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

        pub static ref VEHICLES: [Vehicle; 2] = [
            Vehicle::new(
                (4.03328112e6 - 2.249e6) * KILOGRAM_PER_POUND,
                4500. * SQUARE_METER_PER_SQUARE_FOOT,
                Table2D::new(
                    DRAG_TABLE1.0,
                    DRAG_TABLE1.1,
                    DRAG_TABLE1.2,
                    [StateVariable::Alpha, StateVariable::MachNumber]
                ),
                Table2D::new(
                    LIFT_TABLE1.0,
                    LIFT_TABLE1.1,
                    LIFT_TABLE1.2,
                    [StateVariable::Alpha, StateVariable::MachNumber]
                ),
                Table2D::default(),
                vec![Engine::new(
                    [0., 0.],
                    5472000.0 * NEWTON_PER_POUND_FORCE,
                    439.0,
                    232.5 * SQUARE_METER_PER_SQUARE_FOOT,
                )],
                f64::INFINITY,
            ),
            Vehicle::new(
                (3.57822526e5 - 4.75414027e4) * KILOGRAM_PER_POUND,
                4840. * SQUARE_METER_PER_SQUARE_FOOT,
                Table2D::new(
                    DRAG_TABLE2.0,
                    DRAG_TABLE2.1,
                    DRAG_TABLE2.2,
                    [StateVariable::Alpha, StateVariable::MachNumber]
                ),
                Table2D::new(
                    LIFT_TABLE2.0,
                    LIFT_TABLE2.1,
                    LIFT_TABLE2.2,
                    [StateVariable::Alpha, StateVariable::MachNumber]
                ),
                Table2D::default(),
                vec![Engine::new(
                    [0., 0.],
                    1431000.0 * NEWTON_PER_POUND_FORCE,
                    459.0,
                    154.54 * SQUARE_METER_PER_SQUARE_FOOT,
                )],
                3. * STD_GRAVITY,
            ),
        ];
    }
}

mod steering {
    pub static PITCH_RATES: [f64; 8] = [
        -4.02959110e-1,
        -4.55853620e-1,
        -1.67888963e-1,
        -6.77243251e-1,
        -2.87672429e-1,
        -6.85708451e-2,
        -1.30635729e-1,
        -1.16711775e-1,
    ];
}

mod data {
    use lazy_static::lazy_static;
    use nalgebra::Vector3;

    use crate::constants::*;
    use crate::{transformations::inertial_to_planet, State, Vehicle, EARTH_SPHERICAL};

    use super::{PITCH_RATES, VEHICLES};

    pub struct DataPoint {
        pub time: f64,
        pub time_since_event: f64,
        pub position: Vector3<f64>,
        pub velocity: Vector3<f64>,
        pub mass: f64,
        pub altitude: f64,
        pub temperature: f64,
        pub pressure: f64,
        pub density: f64,
        pub mach_number: f64,
        pub dynamic_pressure: f64,
        pub euler_angles: [f64; 3],
        pub acceleration: Vector3<f64>,
        pub propellant_mass: f64,
        pub massflow: f64,
        pub vehicle_acceleration: Vector3<f64>,
        pub vehicle_acceleration_inertial: Vector3<f64>,
        pub throttle: f64,
        pub thrust_force: Vector3<f64>,
        pub aero_force: Vector3<f64>,
        pub alpha: f64,
        pub vehicle: Vehicle,
        pub steering_coeffs: [f64; 2],
        pub launch: [f64; 3],
    }

    impl DataPoint {
        pub fn to_state(&self) -> State {
            let velocity_planet = EARTH_SPHERICAL.velocity_planet(self.position, self.velocity);
            State::from_values(
                self.time,
                self.time_since_event,
                self.position,
                inertial_to_planet(self.time, EARTH_SPHERICAL.rotation_rate)
                    .transform_vector(&self.position),
                self.altitude,
                EARTH_SPHERICAL.geopotational_altitude(self.position),
                self.velocity,
                velocity_planet,
                velocity_planet,
                self.acceleration,
                self.thrust_force,
                self.aero_force,
                self.vehicle_acceleration,
                self.acceleration - self.vehicle_acceleration_inertial,
                self.mass,
                self.propellant_mass,
                self.massflow,
                self.temperature,
                self.pressure,
                self.density,
                self.mach_number,
                self.dynamic_pressure,
                self.alpha.to_radians(),
                self.euler_angles,
                self.throttle,
            )
        }
    }

    lazy_static! {
        pub static ref DATA_POINTS: [DataPoint; 4] = [
            DataPoint {
                time: 0.,
                time_since_event: 0.,
                mass: 4.03328112e6 * KILOGRAM_PER_POUND,
                position: Vector3::new(3.00354800e6, -1.81429627e7, 9.98490063e6) * METER_PER_FOOT,
                velocity: Vector3::new(1.32300480e3, 2.19022024e2, 0.) * METER_PER_FOOT,
                acceleration: Vector3::new(1.06789869, -6.57146330, 3.61657625) * METER_PER_FOOT,
                altitude: -4.76837158e-7 * METER_PER_FOOT,
                euler_angles: [0., 0., 0.],
                temperature: 5.18670000e2 * KELVIN_PER_RANKIN,
                pressure: 2.11621660e3 * PASCAL_PER_PSF,
                density: 2.37690697e-3 * KILOGRAM_PER_SLUG / CUBIC_METER_PER_CUBIC_FOOT,
                alpha: 0.,
                mach_number: 0.,
                throttle: 1.,
                thrust_force: Vector3::new(4.97997964e6, 0., 0.) * NEWTON_PER_POUND_FORCE,
                dynamic_pressure: 0. * PASCAL_PER_PSF,
                aero_force: Vector3::new(-0., 0., -0.) * NEWTON_PER_POUND_FORCE,
                vehicle: VEHICLES[0].clone(),
                steering_coeffs: [0., 0.],
                propellant_mass: 2.24900000e6 * KILOGRAM_PER_POUND,
                massflow: -1.24646925e4 * KILOGRAM_PER_POUND,
                vehicle_acceleration_inertial: Vector3::new(
                    5.70200852e0,
                    -3.44430413e1,
                    1.89555790e1
                ) * METER_PER_FOOT,
                vehicle_acceleration: Vector3::new(3.97259353e1, 0., 0.) * METER_PER_FOOT,
                launch: [
                    28.5_f64.to_radians(),
                    279.4_f64.to_radians(),
                    90_f64.to_radians()
                ]
            },
            DataPoint {
                time: 1.50000000e1,
                time_since_event: 1.50000000e1,
                mass: 3.84631074e6 * KILOGRAM_PER_POUND,
                position: Vector3::new(3.02352433e6, -1.81404764e7, 9.98534136e6) * METER_PER_FOOT,
                velocity: Vector3::new(1.34110394e3, 1.07998292e2, 6.10765341e1) * METER_PER_FOOT,
                acceleration: Vector3::new(1.32648099, -8.25804904, 4.54117835) * METER_PER_FOOT,
                altitude: 9.33310129e2 * METER_PER_FOOT,
                euler_angles: [0., 0., 0.],
                temperature: 5.15341815e2 * KELVIN_PER_RANKIN,
                pressure: 2.04581341e3 * PASCAL_PER_PSF,
                density: 2.31267089e-3 * KILOGRAM_PER_SLUG / CUBIC_METER_PER_CUBIC_FOOT,
                alpha: -1.48963081e-1,
                mach_number: 1.16179831e-1,
                throttle: 1.,
                thrust_force: Vector3::new(4.99634838e6, 0., 0.) * NEWTON_PER_POUND_FORCE,
                dynamic_pressure: 1.93297185e1 * PASCAL_PER_PSF,
                aero_force: Vector3::new(-1.59202357e4, 0., -1.09857008e3) * NEWTON_PER_POUND_FORCE,
                vehicle: VEHICLES[0].clone(),
                steering_coeffs: [0., 0.],
                propellant_mass: 2.06202961e6 * KILOGRAM_PER_POUND,
                massflow: -1.24646925e4 * KILOGRAM_PER_POUND,
                vehicle_acceleration_inertial: Vector3::new(
                    5.97065742e0,
                    -3.61220822e1,
                    1.98788049e1
                ) * METER_PER_FOOT,
                vehicle_acceleration: Vector3::new(4.16607773e1, 0., -9.18942751e-3)
                    * METER_PER_FOOT,
                launch: [
                    28.5_f64.to_radians(),
                    279.4_f64.to_radians(),
                    90_f64.to_radians()
                ]
            },
            DataPoint {
                time: 4.37456932e2,
                time_since_event: 0.,
                mass: 3.57822526e5 * KILOGRAM_PER_POUND,
                position: Vector3::new(7.02620764e6, -1.73942758e7, 9.94057977e6) * METER_PER_FOOT,
                velocity: Vector3::new(2.23048738e4, 7.85391572e3, -2.22909900e3) * METER_PER_FOOT,
                acceleration: Vector3::new(8.26692668e1, 5.08582088e1, -1.99119531e1)
                    * METER_PER_FOOT,
                altitude: 3.04960868e5 * METER_PER_FOOT,
                euler_angles: [0., 0., -9.66287352e1],
                temperature: 3.41188380e2 * KELVIN_PER_RANKIN,
                pressure: 2.01763300e-3 * PASCAL_PER_PSF,
                density: 3.44501515e-9 * KILOGRAM_PER_SLUG / CUBIC_METER_PER_CUBIC_FOOT,
                alpha: 4.72845235e0,
                mach_number: 2.47287807e1,
                throttle: 7.50268212e-1,
                thrust_force: Vector3::new(1.07363350e6, 0., 0.) * NEWTON_PER_POUND_FORCE,
                dynamic_pressure: 8.63665574e-1 * PASCAL_PER_PSF,
                aero_force: Vector3::new(-1.66106776e2, 0., -6.28680574e2) * NEWTON_PER_POUND_FORCE,
                vehicle: VEHICLES[1].clone(),
                steering_coeffs: [-9.66287352e1, PITCH_RATES[7]],
                propellant_mass: 4.754140278e4 * KILOGRAM_PER_POUND,
                massflow: -2.33907148e3 * KILOGRAM_PER_POUND,
                vehicle_acceleration_inertial: Vector3::new(
                    9.30045549e1,
                    2.52718812e1,
                    -5.28973288e0
                ) * METER_PER_FOOT,
                vehicle_acceleration: Vector3::new(9.65219834e1, 0., -5.65284984e-2)
                    * METER_PER_FOOT,
                launch: [
                    28.5_f64.to_radians(),
                    279.4_f64.to_radians(),
                    90_f64.to_radians()
                ]
            },
            DataPoint {
                time: 4.59264198e2,
                time_since_event: 2.18072658e1,
                mass: 3.10281123e5 * KILOGRAM_PER_POUND,
                position: Vector3::new(7.53212452e6, -1.72106460e7, 9.88707980e6) * METER_PER_FOOT,
                velocity: Vector3::new(2.40870305e4, 8.99928739e3, -2.68459504e3) * METER_PER_FOOT,
                acceleration: Vector3::new(8.07369090e1, 5.41723792e1, -2.18585972e1)
                    * METER_PER_FOOT,
                altitude: 3.03804044e5 * METER_PER_FOOT,
                euler_angles: [0., 0., -9.91738999e1],
                temperature: 3.39250012e2 * KELVIN_PER_RANKIN,
                pressure: 2.14880703e-3 * PASCAL_PER_PSF,
                density: 3.68995220e-9 * KILOGRAM_PER_SLUG / CUBIC_METER_PER_CUBIC_FOOT,
                alpha: 3.39554618e0,
                mach_number: 2.71263292e1,
                throttle: 6.50633622e-1,
                thrust_force: Vector3::new(9.31056380e5, 0., 0.) * NEWTON_PER_POUND_FORCE,
                dynamic_pressure: 1.10682128e0 * PASCAL_PER_PSF,
                aero_force: Vector3::new(-2.13216514e2, 0., -6.17695942e2) * NEWTON_PER_POUND_FORCE,
                vehicle: VEHICLES[1].clone(),
                steering_coeffs: [-9.66287352e1, PITCH_RATES[7]],
                propellant_mass: 0.,
                massflow: -2.02844600e3 * KILOGRAM_PER_POUND,
                vehicle_acceleration_inertial: Vector3::new(
                    9.18182244e1,
                    2.88519537e1,
                    -7.31256468e0
                ) * METER_PER_FOOT,
                vehicle_acceleration: Vector3::new(9.65219787e1, 0., -6.40507842e-2)
                    * METER_PER_FOOT,
                launch: [
                    28.5_f64.to_radians(),
                    279.4_f64.to_radians(),
                    90_f64.to_radians()
                ]
            },
        ];
    }
}
