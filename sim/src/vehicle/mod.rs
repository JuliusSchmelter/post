// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 22.11.23
// Last modified by Tibor Völcker on 17.01.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::{vector, Vector3};
pub use steering::{Angular, Steering};
use utils::tables::linear_interpolation::Table2D;

mod steering;

#[derive(Clone)]
pub struct Vehicle {
    mass: f64,
    reference_area: f64,
    drag_coeff: Table2D,
    lift_coeff: Table2D,
    side_force_coeff: Table2D,
    engines: Vec<Engine>,
    steering: [Option<Steering>; 3],
}

impl Vehicle {
    pub fn new(
        mass: f64,
        reference_area: f64,
        drag_coeff: Table2D,
        lift_coeff: Table2D,
        side_force_coeff: Table2D,
        engines: Vec<Engine>,
        steering: [Option<Steering>; 3],
    ) -> Self {
        Self {
            mass,
            reference_area,
            drag_coeff,
            lift_coeff,
            side_force_coeff,
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

    pub fn aero(&self, alpha: f64, mach: f64, dynamic_pressure: f64) -> Vector3<f64> {
        let cd = self.drag_coeff.at(alpha, mach);
        let cl = self.lift_coeff.at(alpha, mach);
        let cy = self.side_force_coeff.at(alpha, mach);

        let ca = alpha.cos() * cd - alpha.sin() * cl;
        let cn = alpha.sin() * cd + alpha.cos() * cl;

        dynamic_pressure * self.reference_area * vector![-ca, cy, -cn] / self.mass
    }

    pub fn steer(&self, variable: f64) -> Vector3<f64> {
        Vector3::from_iterator(self.steering.iter().map(|steer_opt| {
            if let Some(steer) = steer_opt {
                steer.update(variable)
            } else {
                0.
            }
        }))
    }
}

#[derive(Clone)]
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
    use crate::example_data::example_data;
    use utils::{assert_almost_eq, assert_almost_eq_rel};

    #[test]
    fn test_thrust() {
        let mut data = example_data();

        for data_point in data.iter_mut() {
            print!("Testing {} km altitude ... ", data_point.altitude);
            data_point.vehicle.engines[0].throttle = data_point.throttle;
            let res = data_point.vehicle.thrust(data_point.pressure);
            assert_almost_eq_rel!(res.norm(), data_point.thrust, 0.001);
            println!("ok");
        }
    }

    #[test]
    fn cd() {
        let data = example_data();

        for data_point in data {
            print!("Testing {} km altitude ... ", data_point.altitude);
            assert_almost_eq!(
                data_point
                    .vehicle
                    .drag_coeff
                    .at(data_point.alpha, data_point.mach),
                data_point.cd,
                1e-9
            );
            println!("ok");
        }
    }

    #[test]
    fn cl() {
        let data = example_data();

        for data_point in data {
            print!("Testing {} km altitude ... ", data_point.altitude);
            assert_almost_eq!(
                data_point
                    .vehicle
                    .lift_coeff
                    .at(data_point.alpha, data_point.mach),
                data_point.cl,
                1e-9
            );
            println!("ok");
        }
    }

    #[test]
    fn test_aero() {
        let data = example_data();

        for data_point in data.iter() {
            print!("Testing {} km altitude ... ", data_point.altitude);
            let res = data_point.vehicle.aero(
                data_point.alpha,
                data_point.mach,
                data_point.dynamic_pressure,
            );
            assert_almost_eq_rel!(res[0], data_point.aero_force[0], 0.001);
            assert_almost_eq_rel!(res[1], data_point.aero_force[1], 0.001);
            assert_almost_eq_rel!(res[2], data_point.aero_force[2], 0.001);
            println!("ok");
        }
    }
}
