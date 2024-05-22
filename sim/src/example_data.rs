// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 17.01.24
// Last modified by Tibor Völcker on 22.05.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)
#![cfg(test)]
pub use data::DATA_POINTS;
pub use steering::PITCH_RATES;

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

    use crate::planet::EARTH_SPHERICAL;
    use crate::utils::constants::*;

    use super::PITCH_RATES;

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
        pub vehicle_idx: usize,
        pub steering_coeffs: [f64; 2],
        pub launch: [f64; 3],
        pub max_acceleration: f64,
    }

    impl DataPoint {
        pub fn gravity_acceleration(&self) -> Vector3<f64> {
            self.acceleration - self.vehicle_acceleration_inertial
        }

        pub fn geopotational_altitude(&self) -> f64 {
            EARTH_SPHERICAL.geopotational_altitude(self.position)
        }

        pub fn velocity_planet(&self) -> Vector3<f64> {
            EARTH_SPHERICAL.velocity_planet(self.position, self.velocity)
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
                vehicle_idx: 0,
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
                ],
                max_acceleration: f64::INFINITY,
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
                vehicle_idx: 0,
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
                ],
                max_acceleration: f64::INFINITY,
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
                euler_angles: [0., 0., -9.66287352e1_f64.to_radians()],
                temperature: 3.41188380e2 * KELVIN_PER_RANKIN,
                pressure: 2.01763300e-3 * PASCAL_PER_PSF,
                density: 3.44501515e-9 * KILOGRAM_PER_SLUG / CUBIC_METER_PER_CUBIC_FOOT,
                alpha: 4.72845235e0,
                mach_number: 2.47287807e1,
                throttle: 7.50268212e-1,
                thrust_force: Vector3::new(1.07363350e6, 0., 0.) * NEWTON_PER_POUND_FORCE,
                dynamic_pressure: 8.63665574e-1 * PASCAL_PER_PSF,
                aero_force: Vector3::new(-1.66106776e2, 0., -6.28680574e2) * NEWTON_PER_POUND_FORCE,
                vehicle_idx: 2,
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
                ],
                max_acceleration: 3. * STD_GRAVITY,
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
                euler_angles: [0., 0., -9.91738999e1_f64.to_radians()],
                temperature: 3.39250012e2 * KELVIN_PER_RANKIN,
                pressure: 2.14880703e-3 * PASCAL_PER_PSF,
                density: 3.68995220e-9 * KILOGRAM_PER_SLUG / CUBIC_METER_PER_CUBIC_FOOT,
                alpha: 3.39554618e0,
                mach_number: 2.71263292e1,
                throttle: 6.50633622e-1,
                thrust_force: Vector3::new(9.31056380e5, 0., 0.) * NEWTON_PER_POUND_FORCE,
                dynamic_pressure: 1.10682128e0 * PASCAL_PER_PSF,
                aero_force: Vector3::new(-2.13216514e2, 0., -6.17695942e2) * NEWTON_PER_POUND_FORCE,
                vehicle_idx: 2,
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
                ],
                max_acceleration: 3. * STD_GRAVITY,
            },
        ];
    }
}
