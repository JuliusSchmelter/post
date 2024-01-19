// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 22.11.23
// Last modified by Tibor Völcker on 19.01.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use std::f64::consts::PI;

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
    max_acceleration: f64,
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
        max_acceleration: f64,
    ) -> Self {
        Self {
            mass,
            reference_area,
            drag_coeff,
            lift_coeff,
            side_force_coeff,
            engines,
            steering,
            max_acceleration,
        }
    }

    pub fn thrust(&self, pressure_atmos: f64) -> Vector3<f64> {
        self.engines
            .iter()
            .map(|eng| eng.thrust(pressure_atmos))
            .sum::<Vector3<f64>>()
            / self.mass
    }

    fn auto_max_thrust(&self, angle_aero_thrust: f64, aero: f64) -> f64 {
        // Opposite directions
        if angle_aero_thrust == 0. {
            return self.max_acceleration + aero;
        }

        // Same direction
        if angle_aero_thrust == PI {
            return self.max_acceleration - aero;
        }

        // No intersection possible (arcsin not defined)
        // Aero acceleration too big and gamma too small
        if angle_aero_thrust.sin() * aero > self.max_acceleration {
            panic!("Could not stay in max. acceleration (check aero forces)")
        }

        // Angle between aero forces and sensed acceleration
        let angle_aero_sensed = PI
            - (angle_aero_thrust.sin() * aero / self.max_acceleration).asin()
            - angle_aero_thrust;

        self.max_acceleration * angle_aero_sensed.sin() / angle_aero_thrust.sin()
    }

    pub fn auto_throttle(&self, thrust: Vector3<f64>, aero: Vector3<f64>) -> Vector3<f64> {
        // Thrust needed to reach maximum acceleration
        let max_thrust = self.auto_max_thrust(aero.angle(&thrust), aero.norm());

        let throttle = (max_thrust / thrust.norm()).clamp(0., 1.);

        let throttled_thrust = throttle * thrust;

        // Intersection would require negative thrust
        if (aero + throttled_thrust).norm() > self.max_acceleration {
            panic!("Could not stay in max. acceleration (check aero forces)")
        }

        throttled_thrust
    }

    pub fn aero(&self, alpha: f64, mach: f64, dynamic_pressure: f64) -> Vector3<f64> {
        let cd = self.drag_coeff.at(alpha, mach);
        let cl = self.lift_coeff.at(alpha, mach);
        let cy = self.side_force_coeff.at(alpha, mach);

        let ca = alpha.to_radians().cos() * cd - alpha.to_radians().sin() * cl;
        let cn = alpha.to_radians().sin() * cd + alpha.to_radians().cos() * cl;

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
        let data = example_data();

        for data_point in data {
            print!("Testing {} km altitude ... ", data_point.altitude);
            let thrust = data_point.vehicle.thrust(data_point.pressure);
            let throttled_thrust = data_point
                .vehicle
                .auto_throttle(thrust, data_point.aero_acc);
            assert_almost_eq_rel!(
                throttled_thrust.norm() / thrust.norm(),
                data_point.auto_throttle,
                0.0005
            );
            assert_almost_eq_rel!(
                throttled_thrust.norm(),
                data_point.thrust / data_point.vehicle.mass,
                0.0005
            );
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

        for data_point in data {
            print!("Testing {} km altitude ... ", data_point.altitude);
            let res = data_point.vehicle.aero(
                data_point.alpha,
                data_point.mach,
                data_point.dynamic_pressure,
            );
            assert_almost_eq_rel!(res[0], data_point.aero_acc[0], 0.001);
            assert_almost_eq_rel!(res[1], data_point.aero_acc[1], 0.001);
            assert_almost_eq_rel!(res[2], data_point.aero_acc[2], 0.001);
            println!("ok");
        }
    }
}
