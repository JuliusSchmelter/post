// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 06.12.23
// Last modified by Tibor Völcker on 06.03.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use derive_more::{Deref, DerefMut};
use nalgebra::{Rotation3, Vector3};

use crate::{atmosphere::State as AtmosState, transformations::launch_to_body};

pub enum Axis {
    Roll,
    Pitch,
    Yaw,
}

#[derive(Default)]
pub struct Steering {
    roll: [f64; 4],
    pitch: [f64; 4],
    yaw: [f64; 4],
}

impl Steering {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn update_steering(&mut self, axis: Axis, coeffs: [f64; 3]) -> &Self {
        let mut s = match axis {
            Axis::Roll => self.roll,
            Axis::Pitch => self.pitch,
            Axis::Yaw => self.yaw,
        };
        s[1..].copy_from_slice(&coeffs);
        self
    }

    pub fn init(&mut self, euler_angles: Vector3<f64>) -> &Self {
        self.roll[0] = euler_angles.x;
        self.pitch[0] = euler_angles.y;
        self.yaw[0] = euler_angles.z;

        self
    }
}

#[derive(Default, Deref, DerefMut)]
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
        let euler_angles = Vector3::new(
            Self::calc_coeff(state.time_since_event, self.roll),
            Self::calc_coeff(state.time_since_event, self.pitch),
            Self::calc_coeff(state.time_since_event, self.yaw),
        );

        let inertial_to_body = launch_to_body(
            euler_angles.x.to_radians(),
            euler_angles.y.to_radians(),
            euler_angles.z.to_radians(),
        ) * state.inertial_to_launch;

        State {
            euler_angles: euler_angles.into(),
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
        steering.init(Vector3::new(0., 4., 0.));
        steering.update_steering(Axis::Roll, [3., 2., 1.]);
        let mut state = AtmosState::default();
        state.time = 2.;

        assert_eq!(
            steering.steering(state).euler_angles[1],
            4. + 3. * 2. + 2. * 2_f64.powi(2) + 1. * 2_f64.powi(3)
        )
    }
}
