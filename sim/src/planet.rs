// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 17.11.23
// Last modified by Tibor Völcker on 09.12.23
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use crate::utils::*;
use crate::Atmosphere;
use nalgebra::{vector, Vector3};

pub struct Planet {
    equatorial_radius: f64,
    polar_radius: f64,
    gravitational_parameters: [f64; 4],
    rotation_rate: f64,
    atmosphere: Option<Atmosphere>,
}

impl Planet {
    pub fn earth_spherical(atmosphere: Option<Atmosphere>) -> Self {
        Self {
            equatorial_radius: 2.0925741e7 * METER_PER_FOOT,
            polar_radius: 2.0925741e7 * METER_PER_FOOT,
            // [mu, J_2, J_3, J_4]
            gravitational_parameters: [1.4076539e16 * CUBIC_METER_PER_CUBIC_FOOT, 0., 0., 0.],
            rotation_rate: 7.29211e-5,
            atmosphere,
        }
    }

    pub fn earth_fisher_1960(atmosphere: Option<Atmosphere>) -> Self {
        Self {
            equatorial_radius: 2.0925741e7 * METER_PER_FOOT,
            polar_radius: 2.0855590e7 * METER_PER_FOOT,
            // [mu, J_2, J_3, J_4]
            gravitational_parameters: [
                1.4076539e16 * CUBIC_METER_PER_CUBIC_FOOT,
                1.0823e-3,
                0.,
                0.,
            ],
            rotation_rate: 7.29211e-5,
            atmosphere,
        }
    }

    pub fn earth_smithsonian(atmosphere: Option<Atmosphere>) -> Self {
        Self {
            equatorial_radius: 2.0925741e7 * METER_PER_FOOT,
            polar_radius: 2.0855590e7 * METER_PER_FOOT,
            // [mu, J_2, J_3, J_4]
            gravitational_parameters: [
                1.407645794e16 * CUBIC_METER_PER_CUBIC_FOOT,
                1.082639e-3,
                -2.565e-6,
                -1.608e-6,
            ],
            rotation_rate: 7.29211e-5,
            atmosphere,
        }
    }

    #[allow(non_snake_case)]
    pub fn gravity(&self, position: Vector3<f64>) -> Vector3<f64> {
        let r: f64 = position.norm();
        let R = self.equatorial_radius / r;
        let Z = position.z / r;
        let J = 3. / 2. * self.gravitational_parameters[1];
        let H = 5. / 2. * self.gravitational_parameters[2];
        let D = -35. / 8. * self.gravitational_parameters[3];
        let P = 1.
            + J * R.powi(2) * (1. - 5. * Z.powi(2))
            + H * R.powi(3) / r * (3. - 7. * Z.powi(2)) * position.z
            + D * R.powi(4) * (9. * Z.powi(4) - 6. * Z.powi(2) + 3. / 7.);

        vector![
            -self.mu() * position.x / r.powi(3) * P,
            -self.mu() * position.y / r.powi(3) * P,
            -self.mu() / r.powi(3)
                * ((1. + J * R.powi(2) * (3. - 5. * Z.powi(2))) * position.z
                    + H * R.powi(3) / r
                        * (6. * position.z.powi(2)
                            - 7. * position.z.powi(2) * Z.powi(2)
                            - 3. / 5. * r.powi(2))
                    + D * R.powi(4) * (15. / 7. - 10. * Z.powi(2) + 9. * Z.powi(4)) * position.z)
        ]
    }

    pub fn geopotational_altitude(&self, altitude: f64) -> f64 {
        let avg_altitude = 0.5 * (self.equatorial_radius + self.polar_radius);
        avg_altitude * altitude / (avg_altitude + altitude)
    }

    pub fn mu(&self) -> f64 {
        self.gravitational_parameters[0]
    }
}

#[cfg(test)]
mod tests {
    mod spherical {
        use super::super::*;
        use crate::assert_almost_eq;
        use nalgebra::vector;
        use std::f64::consts::PI;

        #[test]
        fn equatorial_x() {
            let planet = Planet::earth_spherical(None);
            let vec = vector![planet.equatorial_radius, 0., 0.];
            assert_almost_eq!(planet.gravity(vec).norm(), 9.798, 0.0005);
        }

        #[test]
        fn equatorial_xy() {
            let planet = Planet::earth_spherical(None);
            let r = f64::sqrt(planet.equatorial_radius.powi(2) / 2.);
            let vec = vector![r, r, 0.];
            let acc = planet.gravity(vec);
            assert_almost_eq!(acc[0], -f64::sqrt(9.798_f64.powi(2) / 2.), 0.0005);
            assert_almost_eq!(acc[1], -f64::sqrt(9.798_f64.powi(2) / 2.), 0.0005);
            assert_eq!(acc[2], 0.);
        }

        #[test]
        fn polar() {
            let planet = Planet::earth_spherical(None);
            let vec = vector![0., 0., planet.polar_radius];
            assert_almost_eq!(planet.gravity(vec).norm(), 9.798, 0.0005);
        }

        #[test]
        fn sidereal_day() {
            let planet = Planet::earth_fisher_1960(None);
            assert_almost_eq!(2. * PI / planet.rotation_rate, 86164., 0.5);
        }
    }

    mod oblate {
        use super::super::*;
        use crate::assert_almost_eq;
        use nalgebra::vector;
        use std::f64::consts::PI;

        #[test]
        fn equatorial_x() {
            let planet = Planet::earth_fisher_1960(None);
            let vec = vector![planet.equatorial_radius, 0., 0.];
            assert_almost_eq!(planet.gravity(vec).norm(), 9.814, 0.0005);
        }

        #[test]
        fn equatorial_xy() {
            let planet = Planet::earth_fisher_1960(None);
            let r = f64::sqrt(planet.equatorial_radius.powi(2) / 2.);
            let vec = vector![r, r, 0.];
            let acc = planet.gravity(vec);
            assert_almost_eq!(acc[0], -f64::sqrt(9.814_f64.powi(2) / 2.), 0.0005);
            assert_almost_eq!(acc[1], -f64::sqrt(9.814_f64.powi(2) / 2.), 0.0005);
            assert_eq!(acc[2], 0.);
        }

        #[test]
        fn polar() {
            let planet = Planet::earth_fisher_1960(None);
            let vec = vector![0., 0., planet.polar_radius];
            assert_almost_eq!(planet.gravity(vec).norm(), 9.832, 0.0005);
        }

        #[test]
        fn sidereal_day() {
            let planet = Planet::earth_fisher_1960(None);
            assert_almost_eq!(2. * PI / planet.rotation_rate, 86164., 0.5);
        }
    }

    mod smithsonian {
        use super::super::*;
        use crate::assert_almost_eq;
        use nalgebra::vector;
        use std::f64::consts::PI;

        #[test]
        fn equatorial_x() {
            let planet = Planet::earth_smithsonian(None);
            let vec = vector![planet.equatorial_radius, 0., 0.];
            assert_almost_eq!(planet.gravity(vec).norm(), 9.814, 0.0005);
        }

        #[test]
        fn equatorial_xy() {
            let planet = Planet::earth_smithsonian(None);
            let r = f64::sqrt(planet.equatorial_radius.powi(2) / 2.);
            let vec = vector![r, r, 0.];
            let acc = planet.gravity(vec);
            assert_almost_eq!(acc[0], -f64::sqrt(9.814_f64.powi(2) / 2.), 0.0005);
            assert_almost_eq!(acc[1], -f64::sqrt(9.814_f64.powi(2) / 2.), 0.0005);
            assert_ne!(acc[2], 0.);
        }

        #[test]
        fn polar() {
            let planet = Planet::earth_smithsonian(None);
            let vec = vector![0., 0., planet.polar_radius];
            assert_almost_eq!(planet.gravity(vec).norm(), 9.832, 0.0005);
        }

        #[test]
        fn sidereal_day() {
            let planet = Planet::earth_smithsonian(None);
            assert_almost_eq!(2. * PI / planet.rotation_rate, 86164., 0.5);
        }
    }
}
