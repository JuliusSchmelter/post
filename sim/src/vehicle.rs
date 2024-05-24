// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 22.11.23
// Last modified by Tibor Völcker on 24.05.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

//! Defines the [`Vehicle`] struct, which handles all functions
//! regarding the vehicle.

use crate::config::VehicleConfig;
use crate::state::State;
use crate::utils::constants::{NEARLY_ZERO, STD_GRAVITY};
use crate::utils::Table;
use nalgebra::{vector, Vector3};
use serde::Deserialize;
use std::f64::consts::PI;

/// Calculates the third side of a triangle using two sides and
/// an angle.
///
/// This is used for auto-throttling, refer to the
/// [user manual](https://tiborvoelcker.github.io/post/manual.pdf) for more info.
/// Referring to Fig. 6, `a` corresponds to A_SB, and `b` corresponds to A_AB.
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

    // Use the shorter solution of the two possible solutions
    if b > a {
        beta = PI - beta;
    }

    let gamma = PI - beta - alpha;

    Some(a * gamma.sin() / alpha.sin())
}

/// Represents the vehicle.
///
/// Its method use the parameters together with some state variables to
/// calculate derived state variables.
#[derive(Debug, Default, Clone)]
pub struct Vehicle {
    /// Mass of the vehicle without propellant in kg.
    pub structure_mass: f64,
    /// Initial mass of the propellant in kg.
    pub initial_propellant_mass: f64,
    /// Reference area of the vehicle in m^2. Used for aerodynamic calculations.
    reference_area: f64,
    /// Table used to calculate the drag coefficients.
    drag_coeff: Table,
    /// Table used to calculate the lift coefficients.
    lift_coeff: Table,
    /// Table used to calculate the side-force coefficients.
    side_force_coeff: Table,
    /// Engines of the vehicle.
    engines: Vec<Engine>,
}

impl Vehicle {
    /// Updates itself with the new configuration parameters.
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
    /// Calculates the angle-of-attack. `velocity` should be the velocity with
    /// respect to the atmosphere in body frame.
    pub fn alpha(velocity: Vector3<f64>) -> f64 {
        if velocity.x < NEARLY_ZERO {
            if velocity.z < NEARLY_ZERO {
                return 0.;
            }
            return velocity.z.signum() * PI / 2.;
        }

        f64::atan(velocity.z / velocity.x)
    }

    /// Calculates the thrust force of the vehicle at the current throttle
    /// and atmospheric pressure.
    ///
    /// The function simply adds up the thrust of each engine.
    pub fn thrust_force(&self, throttle: f64, pressure_atmos: f64) -> Vector3<f64> {
        throttle
            * self
                .engines
                .iter()
                .map(|eng| eng.thrust(pressure_atmos))
                .sum::<Vector3<f64>>()
    }

    /// Calculates the massflow of the vehicle at the current throttle.
    ///
    /// The function simply adds up the massflow of each engine.
    pub fn massflow(&self, throttle: f64) -> f64 {
        throttle * self.engines.iter().map(|eng| eng.massflow()).sum::<f64>()
    }

    /// Calculates the throttle setting to stay within the specified maximum
    /// acceleration.
    ///
    /// The function first gathers the maximum possible thrust. It then uses
    /// the [`side-side-angle`] algorithm to figure out the required thrust to
    /// reach the maximum allowed acceleration. Lastly it will clamp the
    /// throttle to be between 0 and 1.
    ///
    /// __Attention:__ The function does not guarantee that the throttle will
    /// lead to an allowed acceleration, because it clamps the throttle.
    /// Imagine a huge aerodynamic acceleration: The vehicle could stay within
    /// the allowed acceleration if it compensates the big aerodynamic
    /// acceleration with a huge thrust. As the throttle needs to be clamped,
    /// it could have resulted in an invalid acceleration.
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

    /// Calculates the aerodynamic force on the vehicle.
    ///
    /// The function requires the complete state, as the aerodynamic
    /// coefficients are calculated with tables, which can take any state
    /// variable as input.
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

/// Represents an engine of the vehicle.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Engine {
    /// Thrust vector incidence (angle between thrust vector and body frame)
    /// in rad. First entry is Pitch, second is Yaw.
    incidence: [f64; 2],
    /// Vacuum thrust in N.
    thrust_vac: f64,
    /// Specific impulse in sec.
    isp_vac: f64,
    /// Exit area in m^2
    exit_area: f64,
}

impl Engine {
    /// Calculates the thrust vector using the vacuum thrust, exit area,
    /// atmospheric pressure and incidence angles.
    fn thrust(&self, pressure_atmos: f64) -> Vector3<f64> {
        vector![
            self.incidence[1].cos() * self.incidence[0].cos(),
            self.incidence[1].sin(),
            self.incidence[1].cos() * self.incidence[0].sin()
        ] * (self.thrust_vac - self.exit_area * pressure_atmos)
    }

    /// Calculates the massflow using the vacuum thrust and specific impulse.
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
                Vehicle::alpha(inertial_to_body.transform_vector(&data_point.velocity_planet())),
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
