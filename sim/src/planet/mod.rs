// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 17.11.23
// Last modified by Tibor Völcker on 17.01.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

mod atmosphere;

use std::f64::consts::PI;

pub use atmosphere::Atmosphere;
use nalgebra::{vector, Vector3};
use utils::constants::*;

use self::atmosphere::standard_atmosphere_1962;

pub struct Planet {
    pub equatorial_radius: f64,
    pub polar_radius: f64,
    gravitational_parameters: [f64; 4],
    pub rotation_rate: f64,
    atmosphere: Option<Atmosphere>,
    wind: Option<Vector3<f64>>,
}

pub const EARTH_SPHERICAL: Planet = Planet {
    equatorial_radius: 2.0925741e7 * METER_PER_FOOT,
    polar_radius: 2.0925741e7 * METER_PER_FOOT,
    // [mu, J_2, J_3, J_4]
    gravitational_parameters: [1.4076539e16 * CUBIC_METER_PER_CUBIC_FOOT, 0., 0., 0.],
    rotation_rate: 7.29211e-5,
    atmosphere: None,
    wind: None,
};

pub const EARTH_FISHER_1960: Planet = Planet {
    equatorial_radius: 2.0925741e7 * METER_PER_FOOT,
    polar_radius: 2.0855590e7 * METER_PER_FOOT,
    // [mu, J_2, J_3, J_4]
    gravitational_parameters: [1.4076539e16 * CUBIC_METER_PER_CUBIC_FOOT, 1.0823e-3, 0., 0.],
    rotation_rate: 7.29211e-5,
    atmosphere: None,
    wind: None,
};

pub const EARTH_SMITHSONIAN: Planet = Planet {
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
    atmosphere: None,
    wind: None,
};

impl Planet {
    pub fn add_atmosphere(&mut self, atmosphere: Atmosphere) {
        self.atmosphere = Some(atmosphere);
    }

    pub fn add_wind(&mut self, wind: Vector3<f64>) {
        self.wind = Some(wind);
    }
}

impl Planet {
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

    pub fn mu(&self) -> f64 {
        self.gravitational_parameters[0]
    }

    pub fn geopotational_altitude(&self, altitude: f64) -> f64 {
        let avg_altitude = 0.5 * (self.equatorial_radius + self.polar_radius);
        avg_altitude * altitude / (avg_altitude + altitude)
    }

    pub fn rel_velocity(&self, position: Vector3<f64>, velocity: Vector3<f64>) -> Vector3<f64> {
        velocity - vector![0., 0., self.rotation_rate].cross(&position)
    }

    pub fn atmos_rel_velocity(
        &self,
        position: Vector3<f64>,
        velocity: Vector3<f64>,
    ) -> Vector3<f64> {
        self.rel_velocity(position, velocity) + self.wind.unwrap_or(Vector3::zeros())
    }
}

impl Planet {
    pub fn temperature(&self, position: Vector3<f64>) -> f64 {
        if let Some(atmos) = &self.atmosphere {
            match atmos {
                Atmosphere::StandardAtmosphere1962 => {
                    let geopotential_alt = self.geopotational_altitude(position.norm());
                    standard_atmosphere_1962::temperature(geopotential_alt)
                }
            }
        } else {
            0.
        }
    }

    pub fn pressure(&self, position: Vector3<f64>) -> f64 {
        if let Some(atmos) = &self.atmosphere {
            match atmos {
                Atmosphere::StandardAtmosphere1962 => {
                    let geopotential_alt = self.geopotational_altitude(position.norm());
                    standard_atmosphere_1962::pressure(geopotential_alt)
                }
            }
        } else {
            0.
        }
    }

    pub fn density(&self, position: Vector3<f64>) -> f64 {
        if let Some(atmos) = &self.atmosphere {
            match atmos {
                Atmosphere::StandardAtmosphere1962 => {
                    let geopotential_alt = self.geopotational_altitude(position.norm());
                    standard_atmosphere_1962::density(geopotential_alt)
                }
            }
        } else {
            0.
        }
    }

    pub fn speed_of_sound(&self, position: Vector3<f64>) -> f64 {
        if let Some(atmos) = &self.atmosphere {
            match atmos {
                Atmosphere::StandardAtmosphere1962 => {
                    let geopotential_alt = self.geopotational_altitude(position.norm());
                    standard_atmosphere_1962::speed_of_sound(geopotential_alt)
                }
            }
        } else {
            0.
        }
    }

    pub fn alpha(&self, velocity: Vector3<f64>) -> f64 {
        if velocity.x == 0. {
            return velocity.z.signum() * PI / 2.;
        }
        // From [1]: sin(alpha) = z / sqrt(x^2 + z^2)
        //           cos(alpha) = x / sqrt(x^2 + z^2)
        //               alpha = atan(sin(alpha) / cos(alpha))
        // As far as I can see, is the 'sqrt(x^2 + z^2) term useless
        f64::atan(velocity.z / velocity.x)
    }

    pub fn mach_number(&self, position: Vector3<f64>, velocity: Vector3<f64>) -> f64 {
        if self.speed_of_sound(position) == 0. {
            return 0.;
        }
        self.atmos_rel_velocity(position, velocity).norm() / self.speed_of_sound(position)
    }

    pub fn dynamic_pressure(&self, position: Vector3<f64>, velocity: Vector3<f64>) -> f64 {
        0.5 * self.density(position) * self.atmos_rel_velocity(position, velocity).norm().powi(2)
    }
}

#[cfg(test)]
mod tests {
    mod spherical {
        use super::super::*;
        use nalgebra::vector;
        use std::f64::consts::PI;
        use utils::assert_almost_eq;

        #[test]
        fn equatorial_x() {
            let vec = vector![EARTH_SPHERICAL.equatorial_radius, 0., 0.];
            assert_almost_eq!(EARTH_SPHERICAL.gravity(vec).norm(), 9.798, 0.0005);
        }

        #[test]
        fn equatorial_xy() {
            let r = f64::sqrt(EARTH_SPHERICAL.equatorial_radius.powi(2) / 2.);
            let vec = vector![r, r, 0.];
            let acc = EARTH_SPHERICAL.gravity(vec);
            assert_almost_eq!(acc[0], -f64::sqrt(9.798_f64.powi(2) / 2.), 0.0005);
            assert_almost_eq!(acc[1], -f64::sqrt(9.798_f64.powi(2) / 2.), 0.0005);
            assert_eq!(acc[2], 0.);
        }

        #[test]
        fn polar() {
            let vec = vector![0., 0., EARTH_SPHERICAL.polar_radius];
            assert_almost_eq!(EARTH_SPHERICAL.gravity(vec).norm(), 9.798, 0.0005);
        }

        #[test]
        fn sidereal_day() {
            assert_almost_eq!(2. * PI / EARTH_SPHERICAL.rotation_rate, 86164., 0.5);
        }
    }

    mod oblate {
        use super::super::*;
        use nalgebra::vector;
        use std::f64::consts::PI;
        use utils::assert_almost_eq;

        #[test]
        fn equatorial_x() {
            let vec = vector![EARTH_FISHER_1960.equatorial_radius, 0., 0.];
            assert_almost_eq!(EARTH_FISHER_1960.gravity(vec).norm(), 9.814, 0.0005);
        }

        #[test]
        fn equatorial_xy() {
            let r = f64::sqrt(EARTH_FISHER_1960.equatorial_radius.powi(2) / 2.);
            let vec = vector![r, r, 0.];
            let acc = EARTH_FISHER_1960.gravity(vec);
            assert_almost_eq!(acc[0], -f64::sqrt(9.814_f64.powi(2) / 2.), 0.0005);
            assert_almost_eq!(acc[1], -f64::sqrt(9.814_f64.powi(2) / 2.), 0.0005);
            assert_eq!(acc[2], 0.);
        }

        #[test]
        fn polar() {
            let vec = vector![0., 0., EARTH_FISHER_1960.polar_radius];
            assert_almost_eq!(EARTH_FISHER_1960.gravity(vec).norm(), 9.832, 0.0005);
        }

        #[test]
        fn sidereal_day() {
            assert_almost_eq!(2. * PI / EARTH_FISHER_1960.rotation_rate, 86164., 0.5);
        }
    }

    mod smithsonian {
        use super::super::*;
        use nalgebra::vector;
        use std::f64::consts::PI;
        use utils::assert_almost_eq;

        #[test]
        fn equatorial_x() {
            let vec = vector![EARTH_SMITHSONIAN.equatorial_radius, 0., 0.];
            assert_almost_eq!(EARTH_SMITHSONIAN.gravity(vec).norm(), 9.814, 0.0005);
        }

        #[test]
        fn equatorial_xy() {
            let r = f64::sqrt(EARTH_SMITHSONIAN.equatorial_radius.powi(2) / 2.);
            let vec = vector![r, r, 0.];
            let acc = EARTH_SMITHSONIAN.gravity(vec);
            assert_almost_eq!(acc[0], -f64::sqrt(9.814_f64.powi(2) / 2.), 0.0005);
            assert_almost_eq!(acc[1], -f64::sqrt(9.814_f64.powi(2) / 2.), 0.0005);
            assert_ne!(acc[2], 0.);
        }

        #[test]
        fn polar() {
            let vec = vector![0., 0., EARTH_SMITHSONIAN.polar_radius];
            assert_almost_eq!(EARTH_SMITHSONIAN.gravity(vec).norm(), 9.832, 0.0005);
        }

        #[test]
        fn sidereal_day() {
            assert_almost_eq!(2. * PI / EARTH_SMITHSONIAN.rotation_rate, 86164., 0.5);
        }
    }
}
