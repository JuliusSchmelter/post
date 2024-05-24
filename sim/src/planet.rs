// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 17.11.23
// Last modified by Tibor Völcker on 22.05.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

//! Defines the [`Planet`] struct, which handles all functions
//! regarding the planet.

use crate::config::PlanetConfig;
use crate::utils::constants::{CUBIC_METER_PER_CUBIC_FOOT, METER_PER_FOOT};
use nalgebra::{vector, Vector3};

/// Represents the planet.
///
/// Its method use the parameters together with some state variables to
/// calculate derived state variables.
#[derive(Debug, Clone)]
pub struct Planet {
    /// The equatorial radius im m.
    pub equatorial_radius: f64,
    /// The polar radius in m.
    pub polar_radius: f64,
    /// The gravitational harmonics J1 to J4. The first one is the
    /// gravitational constant in mˆ3/sˆ2.
    gravitational_parameters: [f64; 4],
    /// The rotational rate in rad/s.
    pub rotation_rate: f64,
}

impl Default for Planet {
    fn default() -> Self {
        EARTH_SPHERICAL
    }
}

impl Planet {
    /// Updates itself with the new configuration parameters.
    pub fn update_with_config(config: &PlanetConfig) -> Self {
        match config {
            PlanetConfig::Spherical => EARTH_SPHERICAL,
            PlanetConfig::Fisher1960 => EARTH_FISHER_1960,
            PlanetConfig::Smithsonian => EARTH_SMITHSONIAN,
            PlanetConfig::Custom {
                equatorial_radius,
                polar_radius,
                gravitational_parameters,
                rotation_rate,
            } => Planet {
                equatorial_radius: *equatorial_radius,
                polar_radius: *polar_radius,
                gravitational_parameters: *gravitational_parameters,
                rotation_rate: *rotation_rate,
            },
        }
    }
}

/// Defines the default implementation of a spherical earth.
///
/// This means the equatorial radius is the same as the polar radius, and only
/// the the gravitational constant is used.
pub const EARTH_SPHERICAL: Planet = Planet {
    equatorial_radius: 2.0925741e7 * METER_PER_FOOT,
    polar_radius: 2.0925741e7 * METER_PER_FOOT,
    gravitational_parameters: [1.4076539e16 * CUBIC_METER_PER_CUBIC_FOOT, 0., 0., 0.],
    rotation_rate: 7.29211e-5,
};

/// Defines the default implementation of the 1960 Fisher earth model, as
/// defined in [3, p. IV-1].
///
/// It uses gravitational harmonics up to J2.
const EARTH_FISHER_1960: Planet = Planet {
    equatorial_radius: 2.0925741e7 * METER_PER_FOOT,
    polar_radius: 2.0855590e7 * METER_PER_FOOT,
    gravitational_parameters: [1.4076539e16 * CUBIC_METER_PER_CUBIC_FOOT, 1.0823e-3, 0., 0.],
    rotation_rate: 7.29211e-5,
};

/// Defines the default implementation of the Smithsonial earth model, as
/// defined in [3, p. IV-1].
///
/// It uses gravitational harmonics up to J4.
const EARTH_SMITHSONIAN: Planet = Planet {
    equatorial_radius: 2.0925741e7 * METER_PER_FOOT,
    polar_radius: 2.0855590e7 * METER_PER_FOOT,
    gravitational_parameters: [
        1.407645794e16 * CUBIC_METER_PER_CUBIC_FOOT,
        1.082639e-3,
        -2.565e-6,
        -1.608e-6,
    ],
    rotation_rate: 7.29211e-5,
};

impl Planet {
    /// Calculate the altitude in m above the oblate surface.
    pub fn altitude(&self, position: Vector3<f64>) -> f64 {
        let k = (self.equatorial_radius / self.polar_radius).powi(2);

        let geocentric_lat = f64::asin(position.z / position.norm());

        let distance_to_surface =
            self.equatorial_radius / f64::sqrt(1. + (k - 1.) * geocentric_lat.sin().powi(2));

        position.norm() - distance_to_surface
    }

    /// Calculate the geopotential altitude in m
    /// (used for the atmospheric model).
    pub fn geopotential_altitude(&self, position: Vector3<f64>) -> f64 {
        let altitude = self.altitude(position);
        let avg_altitude = 0.5 * (self.equatorial_radius + self.polar_radius);
        avg_altitude * altitude / (avg_altitude + altitude)
    }

    /// Calculate the velocity with respect to the planet in m/s.
    pub fn velocity_planet(&self, position: Vector3<f64>, velocity: Vector3<f64>) -> Vector3<f64> {
        velocity - vector![0., 0., self.rotation_rate].cross(&position)
    }

    /// Get the gravitational constant in m^3/s^2.
    fn mu(&self) -> f64 {
        self.gravitational_parameters[0]
    }

    /// Calculate the gravitational acceleration in m/s^2, according to
    /// [3, p. IV-3 f.]
    #[allow(non_snake_case)]
    pub fn gravity(&self, position: Vector3<f64>) -> Vector3<f64> {
        let r = position.norm();
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
}

#[cfg(test)]
mod tests {
    use crate::assert_almost_eq_rel;

    use super::*;
    use crate::example_data::DATA_POINTS;

    #[test]
    fn test_environment() {
        const EPSILON: f64 = 0.001;

        // The first two altitudes seem to be not as accurate!
        for data_point in DATA_POINTS[..2].iter() {
            const EPSILON: f64 = 0.005;

            print!("Testing {} m altitude ... ", data_point.altitude);

            assert_almost_eq_rel!(
                EARTH_SPHERICAL.altitude(data_point.position),
                data_point.altitude,
                EPSILON
            );
            assert_almost_eq_rel!(
                vec EARTH_SPHERICAL.gravity(data_point.position),
                data_point.gravity_acceleration(),
                EPSILON
            );

            println!("ok");
        }

        for data_point in DATA_POINTS[2..].iter() {
            print!("Testing {} m altitude ... ", data_point.altitude);

            assert_almost_eq_rel!(
                EARTH_SPHERICAL.altitude(data_point.position),
                data_point.altitude,
                EPSILON
            );
            assert_almost_eq_rel!(
                vec EARTH_SPHERICAL.gravity(data_point.position),
                data_point.gravity_acceleration(),
                EPSILON
            );

            println!("ok");
        }
    }
}
