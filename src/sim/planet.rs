// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 17.11.23
// Last modified by Tibor Völcker on 18.11.23
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::Vector3;

pub trait Planet {
    fn gravity(&self, position: Vector3<f32>) -> Vector3<f32>;
}

pub struct SphericalPlanet {
    pub equatorial_radius: f32,
    pub mu: f32,
}

impl SphericalPlanet {
    const fn new(equatorial_radius: f32, mu: f32) -> Self {
        Self {
            equatorial_radius,
            mu,
        }
    }
}

impl Planet for SphericalPlanet {
    fn gravity(&self, position: Vector3<f32>) -> Vector3<f32> {
        let r = position.norm();
        return -self.mu * position / r.powi(3);
    }
}

pub const SPHERICAL_EARTH: SphericalPlanet = SphericalPlanet::new(6378165.9, 3.986032e14);

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
