// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 22.11.23
// Last modified by Tibor Völcker on 22.05.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use std::f64::consts::PI;

use crate::config::VehicleConfig;
use crate::state::State;
use crate::utils::constants::*;
use crate::utils::Table;
use nalgebra::{vector, Vector3};
use serde::Deserialize;

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
    pub structure_mass: f64,
    pub initial_propellant_mass: f64,
    reference_area: f64,
    drag_coeff: Table,
    lift_coeff: Table,
    side_force_coeff: Table,
    engines: Vec<Engine>,
}

impl Vehicle {
    pub fn update_with_config(&mut self, config: &VehicleConfig) {
        if let Some(config) = config.structure_mass {
            self.structure_mass = config;
        }
        if let Some(config) = config.propellant_mass {
            self.initial_propellant_mass = config;
        }
        if let Some(config) = config.reference_area {
            self.reference_area = config;
        }
        if let Some(config) = &config.drag_coeff {
            self.drag_coeff = config.clone();
        }
        if let Some(config) = &config.lift_coeff {
            self.lift_coeff = config.clone();
        }
        if let Some(config) = &config.side_force_coeff {
            self.side_force_coeff = config.clone();
        }
        if let Some(config) = &config.engines {
            self.engines.clone_from(config);
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

    pub fn auto_throttle(
        &self,
        max_acceleration: f64,
        mass: f64,
        pressure_atmos: f64,
        aero: Vector3<f64>,
    ) -> f64 {
        let max_thrust = self.thrust_force(1., pressure_atmos);

        if max_thrust == Vector3::zeros() {
            // We cannot generate thrust
            return 1.;
        }

        let angle = PI - aero.angle(&max_thrust);

        // Required thrust vector to exactly reach max acceleration
        let opt_req_thrust = side_side_angle(max_acceleration * mass, aero.norm(), angle);

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

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
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
    use super::*;
    use crate::assert_almost_eq_rel;
    use crate::config::PhaseConfig;
    use crate::example_data::DATA_POINTS;
    use crate::transformations::inertial_to_body;

    #[test]
    fn test_force() {
        const EPSILON: f64 = 0.0005;

        let str = include_str!("utils/input.json");

        let configs: Vec<PhaseConfig> = serde_json::from_str(str).unwrap();

        // Cycle through phases to finally build the last one
        let mut vehicles = Vec::new();
        for config in &configs {
            if let Some(config) = &config.vehicle {
                let mut vehicle = Vehicle::default();
                vehicle.update_with_config(config);
                vehicles.push(vehicle);
            }
        }

        for data_point in DATA_POINTS.iter() {
            print!("Testing {} m altitude ... ", data_point.altitude);

            let inertial_to_body = inertial_to_body(data_point.launch, data_point.euler_angles);
            let vehicle = &vehicles[data_point.vehicle_idx];

            assert_almost_eq_rel!(
                vehicle.auto_throttle(
                    data_point.max_acceleration,
                    data_point.mass,
                    data_point.pressure,
                    data_point.aero_force
                ),
                data_point.throttle,
                EPSILON
            );
            assert_almost_eq_rel!(
                vehicle.massflow(data_point.throttle),
                data_point.massflow,
                EPSILON
            );
            assert_almost_eq_rel!(
                data_point.mass - vehicle.structure_mass,
                data_point.propellant_mass,
                EPSILON
            );
            assert_almost_eq_rel!(vec vehicle.thrust_force(data_point.throttle, data_point.pressure), data_point.thrust_force, EPSILON);
            assert_almost_eq_rel!(
                vehicle.alpha(inertial_to_body.transform_vector(&data_point.velocity_planet())),
                data_point.alpha.to_radians(),
                EPSILON
            );

            let state = State {
                alpha: data_point.alpha.to_radians(),
                mach_number: data_point.mach_number,
                aero_force_body: data_point.aero_force,
                dynamic_pressure: data_point.dynamic_pressure,
                ..Default::default()
            };

            assert_almost_eq_rel!(vec vehicle.aero_force(&state), state.aero_force_body, EPSILON);

            println!("ok");
        }
    }
}
