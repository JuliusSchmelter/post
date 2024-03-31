// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 22.11.23
// Last modified by Tibor Völcker on 31.03.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use std::f64::consts::PI;

use crate::constants::{NEARLY_ZERO, STD_GRAVITY};
use crate::state::State;
use crate::utils::Table;
use nalgebra::{vector, Vector3};

fn side_side_angle(a: f64, b: f64, alpha: f64) -> Option<f64> {
    if alpha == 0. {
        if b > a {
            return Some(b - a);
        }
        return Some(a + b);
    }

    if alpha == PI {
        return Some(a - b);
    }

    // No intersection possible (arcsin not defined)
    if alpha.sin() * b > a {
        return None;
    }

    let mut beta = (alpha.sin() * b / a).asin();
    if b > a {
        beta = PI - beta;
    }

    let gamma = PI - beta - alpha;

    Some(a * gamma.sin() / alpha.sin())
}

#[derive(Debug, Default, Clone)]
pub struct Vehicle {
    pub mass: f64,
    reference_area: f64,
    drag_coeff: Table,
    lift_coeff: Table,
    side_force_coeff: Table,
    engines: Vec<Engine>,
    pub max_acceleration: f64,
}

impl Vehicle {
    pub fn new(
        mass: f64,
        reference_area: f64,
        drag_coeff: Table,
        lift_coeff: Table,
        side_force_coeff: Table,
        engines: Vec<Engine>,
        max_acceleration: f64,
    ) -> Self {
        Self {
            mass,
            reference_area,
            drag_coeff,
            lift_coeff,
            side_force_coeff,
            engines,
            max_acceleration,
        }
    }
}

impl Vehicle {
    pub fn alpha(&self, velocity: Vector3<f64>) -> f64 {
        if velocity.x < NEARLY_ZERO {
            if velocity.z < NEARLY_ZERO {
                return 0.;
            }
            return velocity.z.signum() * PI / 2.;
        }

        f64::atan(velocity.z / velocity.x)
    }

    pub fn thrust_force(&self, throttle: f64, pressure_atmos: f64) -> Vector3<f64> {
        throttle
            * self
                .engines
                .iter()
                .map(|eng| eng.thrust(pressure_atmos))
                .sum::<Vector3<f64>>()
    }

    pub fn massflow(&self, throttle: f64) -> f64 {
        throttle * self.engines.iter().map(|eng| eng.massflow()).sum::<f64>()
    }

    pub fn auto_throttle(&self, mass: f64, pressure_atmos: f64, aero: Vector3<f64>) -> f64 {
        let max_thrust = self.thrust_force(1., pressure_atmos);

        if max_thrust == Vector3::zeros() {
            // We cannot generate thrust
            return 1.;
        }

        let angle = PI - aero.angle(&max_thrust);

        // Required thrust vector to exactly reach max acceleration
        let opt_req_thrust = side_side_angle(self.max_acceleration * mass, aero.norm(), angle);

        match opt_req_thrust {
            // The clamping can lead to a throttle which violates the maximum acceleration
            // e.g. if the aero forces are very big
            Some(req_thrust) => (req_thrust / max_thrust.norm()).clamp(0., 1.),
            None => 1.,
        }
    }

    pub fn aero_force(&self, state: &State) -> Vector3<f64> {
        let cd = self.drag_coeff.at_state(state);
        let cl = self.lift_coeff.at_state(state);
        let cy = self.side_force_coeff.at_state(state);

        let ca = state.alpha.cos() * cd - state.alpha.sin() * cl;
        let cn = state.alpha.sin() * cd + state.alpha.cos() * cl;

        let mut aero_force = state.dynamic_pressure * self.reference_area * vector![-ca, cy, -cn];

        // Convert NANs to zeros
        aero_force
            .iter_mut()
            .filter(|i| i.is_nan())
            .for_each(|i| *i = 0.);

        aero_force
    }
}

#[derive(Debug, Clone)]
pub struct Engine {
    // [pitch, yaw]
    incidence: [f64; 2],
    thrust_vac: f64,
    isp_vac: f64,
    exit_area: f64,
}

impl Engine {
    pub fn new(incidence: [f64; 2], thrust_vac: f64, isp_vac: f64, exit_area: f64) -> Self {
        Self {
            incidence,
            thrust_vac,
            isp_vac,
            exit_area,
        }
    }

    fn thrust(&self, pressure_atmos: f64) -> Vector3<f64> {
        vector![
            self.incidence[1].cos() * self.incidence[0].cos(),
            self.incidence[1].sin(),
            self.incidence[1].cos() * self.incidence[0].sin()
        ] * (self.thrust_vac - self.exit_area * pressure_atmos)
    }

    fn massflow(&self) -> f64 {
        -self.thrust_vac / self.isp_vac / STD_GRAVITY
    }
}

#[cfg(test)]
mod tests {
    use crate::assert_almost_eq_rel;
    use crate::example_data::DATA_POINTS;
    use crate::transformations::inertial_to_body;

    #[test]
    fn test_force() {
        const EPSILON: f64 = 0.0005;

        for data_point in DATA_POINTS.iter() {
            print!("Testing {} m altitude ... ", data_point.altitude);

            let state = data_point.to_state();

            let inertial_to_body = inertial_to_body(data_point.launch, state.euler_angles);

            assert_almost_eq_rel!(
                data_point
                    .vehicle
                    .auto_throttle(state.mass, state.pressure, state.aero_force_body),
                state.throttle,
                EPSILON
            );
            assert_almost_eq_rel!(
                data_point.vehicle.massflow(state.throttle),
                state.massflow,
                EPSILON
            );
            assert_almost_eq_rel!(
                state.mass - data_point.vehicle.mass,
                state.propellant_mass,
                EPSILON
            );
            assert_almost_eq_rel!(vec data_point.vehicle.thrust_force(state.throttle, state.pressure), state.thrust_force_body, EPSILON);
            assert_almost_eq_rel!(
                data_point
                    .vehicle
                    .alpha(inertial_to_body.transform_vector(&state.velocity_atmosphere)),
                state.alpha,
                EPSILON
            );
            assert_almost_eq_rel!(vec data_point.vehicle.aero_force(&state), state.aero_force_body, EPSILON);

            println!("ok");
        }
    }
}
