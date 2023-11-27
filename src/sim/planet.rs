// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 17.11.23
// Last modified by Tibor Völcker on 27.11.23
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use crate::sim::utils::*;
use nalgebra::{vector, Vector3};

pub struct Planet {
    equatorial_radius: f32,
    polar_radius: f32,
    gravitational_parameters: [f32; 4],
    rotation_rate: f32,
}

impl Planet {
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

pub const SPHERICAL_EARTH: Planet = Planet {
    equatorial_radius: 2.0925741e7 * METER_PER_FOOT,
    polar_radius: 2.0925741e7 * METER_PER_FOOT,
    // [mu, J_2, J_3, J_4]
    gravitational_parameters: [1.4076539e16 * CUBIC_METER_PER_CUBIC_FOOT, 0., 0., 0.],
    rotation_rate: 7.29211e-5,
};

#[cfg(test)]
mod tests {
    use nalgebra::vector;

    use super::*;

    #[test]
    fn spherical_earth() {
        // test that gravity everywhere at the surface is 9.80
        let vec = vector![6378165.9, 0., 0.];
        assert!(
            (SPHERICAL_EARTH.gravity(vec).norm() - 9.8).abs() < 0.01,
            "Gravity at {:.0} is not roughly 9.80, but '{:.2}'",
            vec,
            SPHERICAL_EARTH.gravity(vec).norm()
        );

        let vec = vector![4510044.4, 4510044.4, 0.];
        let acc = SPHERICAL_EARTH.gravity(vec);
        assert!(
            acc.norm() - 9.8 < 0.01,
            "Gravity at {:.0} is not roughly 9.80, but {:.2}",
            vec,
            SPHERICAL_EARTH.gravity(vec).norm()
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

        let vec = vector![0., 0., 6378165.9];
        assert!(
            SPHERICAL_EARTH.gravity(vec).norm() - 9.8 < 0.01,
            "Gravity at {:.0} is not roughly 9.80, but {:.2}",
            vec,
            SPHERICAL_EARTH.gravity(vec).norm()
        );
    }
}
