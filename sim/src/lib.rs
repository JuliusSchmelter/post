// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 12.11.23
// Last modified by Tibor Völcker on 12.01.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

// allow dead code for now, as it's still WIP
#![allow(dead_code)]

use nalgebra::{Vector3, Vector6};

pub mod integration;
mod planet;
mod transformations;
mod utils;
pub mod vehicle;

pub use integration::Integrator;
pub use planet::{Atmosphere, Planet, EARTH_FISHER_1960, EARTH_SMITHSONIAN, EARTH_SPHERICAL};
pub use vehicle::Vehicle;

use transformations::Transformations;

pub struct Simulation {
    pub time: f64,
    state: Vector6<f64>,
    vehicle: Vehicle,
    planet: Planet,
    transformations: Transformations,
    integrator: Integrator,
    stepsize: f64,
}

impl Simulation {
    pub fn new(vehicle: Vehicle, planet: Planet, stepsize: f64, launch: [f64; 3]) -> Self {
        Simulation {
            time: 0.,
            state: Vector6::zeros(),
            vehicle,
            planet,
            transformations: Transformations::new(launch),
            integrator: Integrator::RK4,
            stepsize,
        }
    }

    fn system(&self, time: f64, state: &Vector6<f64>) -> Vector6<f64> {
        let (pos, vel) = Self::split_state(*state);
        let attitude = self.vehicle.steer(time);

        let bi = self
            .transformations
            .inertial_to_body(attitude.x, attitude.y, attitude.z)
            .transpose();

        let pressure = self.planet.pressure(pos);
        let thrust = self.vehicle.thrust(pressure);

        let gravity = self.planet.gravity(state.fixed_rows::<3>(0).into());

        // r_dot_I = V_I
        let r_dot = vel;
        // V_dot_I = [IB]^-1 [A_TB + A_AB] + G_I
        let v_dot = bi.transform_vector(&thrust) + gravity;

        Vector6::from_row_slice(&[r_dot.as_slice(), (v_dot).as_slice()].concat())
    }

    pub fn step(&mut self) {
        self.state = self.integrator.step(
            |time, state| self.system(time, state),
            self.time,
            self.state,
            self.stepsize,
        );
        self.time += self.stepsize;
    }
}

impl Simulation {
    pub fn position(&self) -> Vector3<f64> {
        self.state.fixed_rows::<3>(0).into()
    }

    pub fn set_position(&mut self, position: &[f64]) {
        self.state.fixed_rows_mut::<3>(0).copy_from_slice(position);
    }

    pub fn velocity(&self) -> Vector3<f64> {
        self.state.fixed_rows::<3>(3).into()
    }

    pub fn set_velocity(&mut self, velocity: &[f64]) {
        self.state.fixed_rows_mut::<3>(3).copy_from_slice(velocity);
    }

    fn split_state(state: Vector6<f64>) -> (Vector3<f64>, Vector3<f64>) {
        (
            state.fixed_rows::<3>(0).into(),
            state.fixed_rows::<3>(3).into(),
        )
    }
}
