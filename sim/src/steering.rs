// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 06.12.23
// Last modified by Tibor Völcker on 06.03.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use derive_more::{Deref, DerefMut};
use nalgebra::{Rotation3, Vector3};

use crate::{atmosphere::State as AtmosState, transformations::launch_to_body};

pub enum Axis {
    Roll,
    Yaw,
    Pitch,
}

#[derive(Debug, Default)]
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

    pub fn init(&mut self, euler_angles: Vector3<f64>) -> &Self {
        self.roll[0] = euler_angles.x;
        self.yaw[0] = euler_angles.y;
        self.pitch[0] = euler_angles.z;

        self
    }
}

#[derive(Debug, Default, Deref, DerefMut)]
pub struct State {
    #[deref]
    #[deref_mut]
    child_state: AtmosState,
    pub euler_angles: [f64; 3],
    pub inertial_to_body: Rotation3<f64>,
    pub body_to_inertial: Rotation3<f64>,
}

impl Steering {
    fn calc_coeff(var: f64, coeffs: [f64; 4]) -> f64 {
        coeffs
            .iter()
            .enumerate()
            .map(|(i, coeff)| coeff * var.powi(i.try_into().unwrap()))
            .sum()
    }

    pub fn steering(&self, state: AtmosState) -> State {
        let euler_angles = [
            Self::calc_coeff(state.time_since_event, self.roll),
            Self::calc_coeff(state.time_since_event, self.yaw),
            Self::calc_coeff(state.time_since_event, self.pitch),
        ];

        let inertial_to_body = launch_to_body(
            euler_angles[0].to_radians(),
            euler_angles[1].to_radians(),
            euler_angles[2].to_radians(),
        ) * state.inertial_to_launch;

        State {
            euler_angles,
            inertial_to_body,
            body_to_inertial: inertial_to_body.transpose(),
            child_state: state,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn angular_polynomials() {
        let mut steering = Steering::new();
        steering.init(Vector3::new(0., 0., 4.));
        steering.update_steering(Axis::Pitch, [3., 2., 1.]);
        let mut state = AtmosState::default();
        state.time_since_event = 2.;

        assert_eq!(
            steering.steering(state).euler_angles[2],
            4. + 3. * 2. + 2. * 2_f64.powi(2) + 1. * 2_f64.powi(3)
        )
    }
}
