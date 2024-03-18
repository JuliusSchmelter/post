// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 17.11.23
// Last modified by Tibor Völcker on 18.03.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use derive_more::{Deref, DerefMut};
use nalgebra::{vector, Rotation3, Vector3};
use utils::constants::*;

use crate::{
    state::PrimaryState,
    transformations::{inertial_to_launch, inertial_to_planet},
};

#[derive(Debug, Clone)]
pub struct Planet {
    pub equatorial_radius: f64,
    pub polar_radius: f64,
    gravitational_parameters: [f64; 4],
    pub rotation_rate: f64,
    pub launch: [f64; 3],
}

pub const EARTH_SPHERICAL: Planet = Planet {
    equatorial_radius: 2.0925741e7 * METER_PER_FOOT,
    polar_radius: 2.0925741e7 * METER_PER_FOOT,
    // [mu, J_2, J_3, J_4]
    gravitational_parameters: [1.4076539e16 * CUBIC_METER_PER_CUBIC_FOOT, 0., 0., 0.],
    rotation_rate: 7.29211e-5,
    launch: [0., 0., 0.],
};

pub const EARTH_FISHER_1960: Planet = Planet {
    equatorial_radius: 2.0925741e7 * METER_PER_FOOT,
    polar_radius: 2.0855590e7 * METER_PER_FOOT,
    // [mu, J_2, J_3, J_4]
    gravitational_parameters: [1.4076539e16 * CUBIC_METER_PER_CUBIC_FOOT, 1.0823e-3, 0., 0.],
    rotation_rate: 7.29211e-5,
    launch: [0., 0., 0.],
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
    launch: [0., 0., 0.],
};

#[derive(Debug, Default, Deref, DerefMut, Clone)]
pub struct State {
    #[deref]
    #[deref_mut]
    child_state: PrimaryState,
    pub position_planet: Vector3<f64>,
    pub inertial_to_launch: Rotation3<f64>,
    pub altitude: f64,
    pub geopotential_altitude: f64,
    pub rel_velocity: Vector3<f64>,
    pub gravity_acceleration: Vector3<f64>,
}

impl Planet {
    pub fn environment(&self, state: PrimaryState) -> State {
        State {
            position_planet: inertial_to_planet(state.time, self.rotation_rate)
                .transform_vector(&state.position),
            inertial_to_launch: inertial_to_launch(self.launch[0], self.launch[1], self.launch[2]),
            altitude: self.altitude(state.position),
            geopotential_altitude: self.geopotational_altitude(state.position),
            rel_velocity: self.rel_velocity(state.position, state.velocity),
            gravity_acceleration: self.gravity(state.position),
            child_state: state,
        }
    }

    pub fn altitude(&self, position: Vector3<f64>) -> f64 {
        let k = (self.equatorial_radius / self.polar_radius).powi(2);

        let geocentric_lat = f64::asin(position.z / position.norm());

        let distance_to_surface =
            self.equatorial_radius / f64::sqrt(1. + (k - 1.) * geocentric_lat.sin().powi(2));

        position.norm() - distance_to_surface
    }

    pub fn geopotational_altitude(&self, position: Vector3<f64>) -> f64 {
        let altitude = self.altitude(position);
        let avg_altitude = 0.5 * (self.equatorial_radius + self.polar_radius);
        avg_altitude * altitude / (avg_altitude + altitude)
    }

    pub fn rel_velocity(&self, position: Vector3<f64>, velocity: Vector3<f64>) -> Vector3<f64> {
        velocity - vector![0., 0., self.rotation_rate].cross(&position)
    }

    pub fn mu(&self) -> f64 {
        self.gravitational_parameters[0]
    }

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
    use std::ops::Deref;
    use utils::assert_almost_eq_rel;

    use super::*;
    use crate::example_data::DATA_POINTS;

    #[test]
    fn test_environment() {
        const EPSILON: f64 = 0.001;

        // The first two altitudes seem to be not as accurate!
        for data_point in DATA_POINTS[..2].iter() {
            const EPSILON: f64 = 0.005;

            print!("Testing {} m altitude ... ", data_point.altitude);

            let state = data_point.to_state();
            let target = state.deref().deref().deref();
            let input = target.deref();

            let output = EARTH_SPHERICAL.environment(input.clone());

            assert_almost_eq_rel!(output.altitude, target.altitude, EPSILON);
            assert_almost_eq_rel!(
                vec output.gravity_acceleration,
                target.gravity_acceleration,
                EPSILON
            );

            println!("ok");
        }

        for data_point in DATA_POINTS[2..].iter() {
            print!("Testing {} m altitude ... ", data_point.altitude);

            let state = data_point.to_state();
            let target = state.deref().deref().deref();
            let input = target.deref();

            let output = EARTH_SPHERICAL.environment(input.clone());

            assert_almost_eq_rel!(output.altitude, target.altitude, EPSILON);
            assert_almost_eq_rel!(
                vec output.gravity_acceleration,
                target.gravity_acceleration,
                EPSILON
            );

            println!("ok");
        }
    }
}
