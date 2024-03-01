// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 06.12.23
// Last modified by Tibor Völcker on 01.03.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

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

pub struct State {
    pub time: f64,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub mass: f64,
    pub altitude: f64,
    pub geopotential_altitude: f64,
    pub rel_velocity: Vector3<f64>,
    pub atmos_rel_velocity: Vector3<f64>,
    pub temperature: f64,
    pub pressure: f64,
    pub density: f64,
    pub speed_of_sound: f64,
    pub mach_number: f64,
    pub dynamic_pressure: f64,
    pub euler_angles: [f64; 3],
    pub inertial_to_body: Rotation3<f64>,
    pub body_to_inertial: Rotation3<f64>,
}

impl Steering {
    pub fn steering(&self, state: &AtmosState) -> State {
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
            time: state.time,
            position: state.position,
            velocity: state.velocity,
            mass: state.mass,
            altitude: state.altitude,
            geopotential_altitude: state.geopotential_altitude,
            rel_velocity: state.rel_velocity,
            atmos_rel_velocity: state.atmos_rel_velocity,
            temperature: state.temperature,
            pressure: state.pressure,
            density: state.density,
            speed_of_sound: state.speed_of_sound,
            mach_number: state.mach_number,
            dynamic_pressure: state.dynamic_pressure,
            euler_angles: euler_angles.into(),
            inertial_to_body,
            body_to_inertial: inertial_to_body.transpose(),
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
        let state = AtmosState {
            time: 2.,
            ..Default::default()
        };

        assert_eq!(
            steering.steering(&state).euler_angles[1],
            4. + 3. * 2. + 2. * 2_f64.powi(2) + 1. * 2_f64.powi(3)
        )
    }
}
