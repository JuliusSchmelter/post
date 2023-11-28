// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 17.11.23
// Last modified by Tibor Völcker on 28.11.23
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use crate::sim::utils::*;
use nalgebra::{vector, Vector3};

use super::atmosphere::Atmosphere;

pub struct Planet {
    equatorial_radius: f32,
    polar_radius: f32,
    gravitational_parameters: [f32; 4],
    rotation_rate: f32,
    atmosphere: Option<Atmosphere>,
}

impl Planet {
    pub fn earth_spherical(atmosphere: Option<Atmosphere>) -> Self {
        return Self {
            equatorial_radius: 2.0925741e7 * METER_PER_FOOT,
            polar_radius: 2.0925741e7 * METER_PER_FOOT,
            // [mu, J_2, J_3, J_4]
            gravitational_parameters: [1.4076539e16 * CUBIC_METER_PER_CUBIC_FOOT, 0., 0., 0.],
            rotation_rate: 7.29211e-5,
            atmosphere,
        };
    }

    pub fn earth_fisher_1960(atmosphere: Option<Atmosphere>) -> Self {
        return Self {
            equatorial_radius: 2.0925741e7 * METER_PER_FOOT,
            polar_radius: 2.0855590 * METER_PER_FOOT,
            // [mu, J_2, J_3, J_4]
            gravitational_parameters: [
                1.4076539e16 * CUBIC_METER_PER_CUBIC_FOOT,
                1.0823e-3,
                0.,
                0.,
            ],
            rotation_rate: 7.29211e-5,
            atmosphere,
        };
    }

    pub fn earth_smithsonian(atmosphere: Option<Atmosphere>) -> Self {
        return Self {
            equatorial_radius: 2.0925741e7 * METER_PER_FOOT,
            polar_radius: 2.0855590 * METER_PER_FOOT,
            // [mu, J_2, J_3, J_4]
            gravitational_parameters: [
                1.407645794e16 * CUBIC_METER_PER_CUBIC_FOOT,
                1.082639e-3,
                -2.565e-6,
                -1.608e-6,
            ],
            rotation_rate: 7.29211e-5,
            atmosphere,
        };
    }

    pub fn gravity(&self, position: Vector3<f32>) -> Vector3<f32> {
        let r: f32 = position.norm();
        let R = self.equatorial_radius / r;
        let Z = position.z / r;
        let J = 3. / 2. * self.gravitational_parameters[1];
        let H = 5. / 2. * self.gravitational_parameters[2];
        let D = -35. / 8. * self.gravitational_parameters[3];
        let P = 1.
            + J * R.powi(2) * (1. - 5. * Z.powi(2))
            + H * R.powi(3) / r * (3. - 7. * Z.powi(2)) * position.z
            + D * R.powi(4) * (9. * Z.powi(4) - 6. * Z.powi(2) + 3. / 7.);

        return vector![
            -self.mu() * position.x / r.powi(3) * P,
            -self.mu() * position.y / r.powi(3) * P,
            -self.mu() / r.powi(3)
                * ((1. + J * R.powi(2) * (3. - 5. * Z.powi(2))) * position.z
                    + H * R.powi(3) / r
                        * (6. * position.z.powi(2)
                            - 7. * position.z.powi(2) * Z.powi(2)
                            - 3. / 5. * r.powi(2))
                    + D * R.powi(4) * (15. / 7. - 10. * Z.powi(2) + 9. * Z.powi(4)) * position.z)
        ];
    }

    pub fn mu(&self) -> f32 {
        return self.gravitational_parameters[0];
    }
}

#[cfg(test)]
mod tests {
    mod spherical {
        use super::super::*;
        use nalgebra::vector;

        #[test]
        fn equatorial_x() {
            // test that gravity everywhere at the surface is 9.80
            let vec = vector![6378165.9, 0., 0.];
            let planet = Planet::earth_spherical(None);
            assert!(
                (planet.gravity(vec).norm() - 9.8).abs() < 0.01,
                "Gravity at {:.0} is not roughly 9.80, but '{:.2}'",
                vec,
                planet.gravity(vec).norm()
            );
        }

        #[test]
        fn equatorial_xy() {
            let planet = Planet::earth_spherical(None);
            let vec = vector![4510044.4, 4510044.4, 0.];
            let acc = planet.gravity(vec);
            assert!(
                acc.norm() - 9.8 < 0.01,
                "Gravity at {:.0} is not roughly 9.80, but {:.2}",
                vec,
                planet.gravity(vec).norm()
            );
            assert!(
                acc[0] < 0.,
                "Gravity at {:.0} has wrong entry at position 0: {:.3} (should be < 0)",
                vec,
                acc[0]
            );
            assert!(
                acc[1] < 0.,
                "Gravity at {:.0} has wrong entry at position 1: {:.3} (should be < 0)",
                vec,
                acc[1]
            );
            assert_eq!(
                acc[2], 0.,
                "Gravity at {:.0} has wrong entry at position 2: {:.3} (should be 0)",
                vec, acc[2]
            );
        }

        #[test]
        fn polar() {
            let planet = Planet::earth_spherical(None);
            let vec = vector![0., 0., 6378165.9];
            assert!(
                planet.gravity(vec).norm() - 9.8 < 0.01,
                "Gravity at {:.0} is not roughly 9.80, but {:.2}",
                vec,
                planet.gravity(vec).norm()
            );
        }
    }
}
