// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 06.12.23
// Last modified by Tibor Völcker on 20.03.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use crate::State;

pub enum Axis {
    Roll,
    Yaw,
    Pitch,
}

#[derive(Debug, Default, Clone)]
pub struct Steering {
    roll: [f64; 4],
    yaw: [f64; 4],
    pitch: [f64; 4],
}

impl Steering {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn update_steering(&mut self, axis: Axis, coeffs: [f64; 3]) -> &Self {
        let s = match axis {
            Axis::Roll => &mut self.roll,
            Axis::Yaw => &mut self.yaw,
            Axis::Pitch => &mut self.pitch,
        };
        (*s)[1..].copy_from_slice(&coeffs);
        self
    }

    pub fn init(&mut self, euler_angles: [f64; 3]) -> &Self {
        self.roll[0] = euler_angles[0];
        self.yaw[0] = euler_angles[1];
        self.pitch[0] = euler_angles[2];

        self
    }
}

impl Steering {
    fn calc_coeff(var: f64, coeffs: [f64; 4]) -> f64 {
        coeffs
            .iter()
            .enumerate()
            .map(|(i, coeff)| coeff * var.powi(i.try_into().unwrap()))
            .sum()
    }

    pub fn steering(&self, state: &mut State) {
        state.euler_angles = [
            Self::calc_coeff(state.time_since_event, self.roll),
            Self::calc_coeff(state.time_since_event, self.yaw),
            Self::calc_coeff(state.time_since_event, self.pitch),
        ];
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;
    use utils::assert_almost_eq_rel;

    use super::*;
    use crate::example_data::DATA_POINTS;

    #[test]
    fn test_steering() {
        const EPSILON: f64 = 0.001;

        let mut steer = Steering::new();

        for data_point in DATA_POINTS.iter() {
            print!("Testing {} m altitude ... ", data_point.altitude);

            steer.init([0., 0., data_point.steering_coeffs[0]]);
            steer.update_steering(Axis::Pitch, [data_point.steering_coeffs[1], 0., 0.]);

            let state = data_point.to_state();
            let mut input = state.clone();

            steer.steering(&mut input);

            let output_euler = Vector3::from_column_slice(&input.euler_angles);
            let target_euler = Vector3::from_column_slice(&state.euler_angles);

            assert_almost_eq_rel!(vec output_euler, target_euler, EPSILON);

            println!("ok");
        }
    }
}
