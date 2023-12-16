// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 22.11.23
// Last modified by Tibor Völcker on 16.12.23
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::{vector, Vector3};
pub use steering::{Angular, Steering};

mod steering;

pub struct Vehicle {
    mass: f64,
    engines: Vec<Engine>,
    steering: Option<Steering>,
}

impl Vehicle {
    pub fn new(mass: f64, engines: Vec<Engine>, steering: Option<Steering>) -> Self {
        Self {
            mass,
            engines,
            steering,
        }
    }

    pub fn thrust(&self, pressure_atmos: f64) -> Vector3<f64> {
        self.engines
            .iter()
            .map(|eng| eng.thrust(pressure_atmos))
            .sum::<Vector3<f64>>()
            / self.mass
    }
}

pub struct Engine {
    // [pitch, yaw]
    incidence: [f64; 2],
    throttle: f64,
    thrust_vac: f64,
    exit_area: f64,
}

impl Engine {
    pub fn new(incidence: [f64; 2], thrust_vac: f64, exit_area: f64) -> Self {
        Self {
            incidence,
            throttle: 1.,
            thrust_vac,
            exit_area,
        }
    }

    fn thrust(&self, pressure_atmos: f64) -> Vector3<f64> {
        vector![
            self.incidence[1].cos() * self.incidence[0].cos(),
            self.incidence[1].sin(),
            self.incidence[1].cos() * self.incidence[0].sin()
        ] * (self.throttle * self.thrust_vac - self.exit_area * pressure_atmos)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::*;
    use crate::Atmosphere;
    use crate::EARTH_SPHERICAL;
    use crate::{assert_almost_eq_rel, utils::NEWTON_PER_POUND_FORCE};
    use nalgebra::vector;

    const THRUST_DATA_SS_EXAMPLE1: [[f64; 3]; 2] = [
        // values from SS example: [2] p. 277
        // [altitude, throttle, thrust]
        [
            -4.76837158e-7 * METER_PER_FOOT,
            1.,
            4.97997964e6 * NEWTON_PER_POUND_FORCE,
        ],
        [
            9.33310129e2 * METER_PER_FOOT,
            1.,
            4.99634838e6 * NEWTON_PER_POUND_FORCE,
        ],
    ];

    const THRUST_DATA_SS_EXAMPLE2: [[f64; 3]; 2] = [
        // values from SS example: [2] p. 277
        // [altitude, throttle, thrust]
        [
            3.04960868e5 * METER_PER_FOOT,
            7.50268212e-1,
            1.07363350e6 * NEWTON_PER_POUND_FORCE,
        ],
        [
            3.03804044e5 * METER_PER_FOOT,
            6.50633622e-1,
            9.31056380e5 * NEWTON_PER_POUND_FORCE,
        ],
    ];

    #[test]
    fn test_thrust() {
        let mut planet = EARTH_SPHERICAL;
        planet.add_atmosphere(Atmosphere::StandardAtmosphere1962);
        let engine = Engine::new(
            [0., 0.],
            5472000.0 * NEWTON_PER_POUND_FORCE,
            232.5 * SQUARE_METER_PER_SQUARE_FOOT,
        );
        let mut vehicle = Vehicle::new(1., vec![engine], None);

        for data_point in THRUST_DATA_SS_EXAMPLE1.iter() {
            print!("Testing {} km altitude ... ", data_point[0]);
            vehicle.engines[0].throttle = data_point[1];
            let res = vehicle.thrust(planet.pressure(vector![data_point[0], 0., 0.]));
            assert_almost_eq_rel!(res.norm(), data_point[2], 0.05);
            println!("ok");
        }

        vehicle.engines[0].thrust_vac = 1431000.0 * NEWTON_PER_POUND_FORCE;
        vehicle.engines[0].exit_area = 154.54 * SQUARE_METER_PER_SQUARE_FOOT;
        for data_point in THRUST_DATA_SS_EXAMPLE2.iter() {
            print!("Testing {} km altitude ... ", data_point[0]);
            vehicle.engines[0].throttle = data_point[1];
            let res = vehicle.thrust(planet.pressure(vector![data_point[0], 0., 0.]));
            assert_almost_eq_rel!(res.norm(), data_point[2], 0.05);
            println!("ok");
        }
    }
}
