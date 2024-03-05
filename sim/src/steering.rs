// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 06.12.23
// Last modified by Tibor Völcker on 05.03.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use derive_more::{Deref, DerefMut};
use nalgebra::{Rotation3, Vector3};

use crate::{atmosphere::State as AtmosState, transformations::launch_to_body};

#[derive(Default)]
pub struct Steering {
    steering: [SteeringModel; 3],
}

#[derive(Default)]
enum SteeringModel {
    #[default]
    NoSteering,
    AngularPolynomials([f64; 4]),
}

impl Steering {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_steering(&mut self, idx: usize, polynomials: [f64; 4]) -> &Self {
        self.steering[idx] = SteeringModel::AngularPolynomials(polynomials);
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
    pub fn steering(&self, state: AtmosState) -> State {
        let euler_angles = Vector3::from_iterator(self.steering.iter().map(|steering_model| {
            match steering_model {
                SteeringModel::AngularPolynomials(coeffs) => (0..4)
                    .map(|i| coeffs[i] * state.time.powi(i.try_into().unwrap()))
                    .sum(),
                SteeringModel::NoSteering => 0.,
            }
        }));

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
        steering.add_steering(1, [4., 3., 2., 1.]);
        let mut state = AtmosState::default();
        state.time = 2.;

        assert_eq!(
            steering.steering(state).euler_angles[1],
            4. + 3. * 2. + 2. * 2_f64.powi(2) + 1. * 2_f64.powi(3)
        )
    }
}
