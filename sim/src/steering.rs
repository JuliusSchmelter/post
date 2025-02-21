// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 06.12.23
// Last modified by Tibor Völcker on 22.05.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

//! Defines the [`Steering`] struct which handles the vehicles orientation.

use crate::{config::SteeringConfig, state::StateVariable, State};

/// Handles the vehicle orientation for each axis.
///
/// It calculates the orientation with cubic polynomials using 4 coefficients
/// and a chosen state variables.
/// The coefficients are stored in ascending order: c0 + c1*y + c2*y^2 + c3*y^3
#[derive(Debug, Default, Clone)]
pub struct Steering {
    /// State variable (unit X) and coefficients in °/X for the roll axis.
    roll: (StateVariable, [f64; 4]),
    /// State variable (unit X) and coefficients in °/X for the yaw axis.
    yaw: (StateVariable, [f64; 4]),
    /// State variable (unit X) and coefficients in °/X for the pitch axis.
    pitch: (StateVariable, [f64; 4]),
}

impl Steering {
    /// Updates itself with the new configuration parameters.
    ///
    /// The configuration parameters only set the latter 3 coefficients, while
    /// the first is set as the last orientation of the previous phase
    /// (see [`Steering::init`]).
    pub fn update_with_config(&mut self, config: &SteeringConfig) {
        if let Some(config) = config.roll {
            self.roll.0 = config.0;
            self.roll.1[1..].copy_from_slice(&config.1);
        }
        if let Some(config) = config.yaw {
            self.yaw.0 = config.0;
            self.yaw.1[1..].copy_from_slice(&config.1);
        }
        if let Some(config) = config.pitch {
            self.pitch.0 = config.0;
            self.pitch.1[1..].copy_from_slice(&config.1);
        }
    }

    /// Sets the first coefficient with their euler angles in rad.
    /// The euler angles are in the order: Roll, Yaw, Pitch.
    pub fn init(&mut self, euler_angles: [f64; 3]) -> &Self {
        self.roll.1[0] = euler_angles[0].to_degrees();
        self.yaw.1[0] = euler_angles[1].to_degrees();
        self.pitch.1[0] = euler_angles[2].to_degrees();

        self
    }
}

impl Steering {
    /// Calculates one angle in ° using some state variable and the
    /// steering coefficients.
    fn calc_coeff(var: f64, coeffs: [f64; 4]) -> f64 {
        coeffs
            .iter()
            .enumerate()
            .map(|(i, coeff)| coeff * var.powi(i.try_into().unwrap()))
            .sum()
    }

    /// Calculates the euler angles in rad in the order: Roll, Pitch, Yaw.
    ///
    /// Calls [`Steering::calc_coeff`] for each axis.
    pub fn euler_angles(&self, state: &State) -> [f64; 3] {
        [
            Self::calc_coeff(self.roll.0.get_value(state), self.roll.1).to_radians(),
            Self::calc_coeff(self.yaw.0.get_value(state), self.yaw.1).to_radians(),
            Self::calc_coeff(self.pitch.0.get_value(state), self.pitch.1).to_radians(),
        ]
    }
}

#[cfg(test)]
mod tests {
    use crate::assert_almost_eq_rel;
    use nalgebra::Vector3;

    use super::*;
    use crate::example_data::DATA_POINTS;

    #[test]
    fn test_steering() {
        const EPSILON: f64 = 0.001;

        let mut steer = Steering::default();

        for data_point in DATA_POINTS.iter() {
            print!("Testing {} m altitude ... ", data_point.altitude);

            steer.init([0., 0., data_point.steering_coeffs[0].to_radians()]);

            steer.update_with_config(&SteeringConfig {
                roll: None,
                yaw: None,
                pitch: Some((
                    StateVariable::TimeSinceEvent,
                    [data_point.steering_coeffs[1], 0., 0.],
                )),
            });

            let state = State {
                time_since_event: data_point.time_since_event,
                euler_angles: data_point.euler_angles,
                ..Default::default()
            };

            let output_euler = Vector3::from_column_slice(&steer.euler_angles(&state));
            let target_euler = Vector3::from_column_slice(&state.euler_angles);

            assert_almost_eq_rel!(vec output_euler, target_euler, EPSILON);

            println!("ok");
        }
    }
}
