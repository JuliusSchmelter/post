// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 12.11.23
// Last modified by Tibor Völcker on 11.02.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

// allow dead code for now, as it's still WIP
#![allow(dead_code)]

#[cfg(test)]
mod example_data;
pub mod integration;
mod planet;
mod state;
mod transformations;
pub mod vehicle;

pub use integration::Integrator;
use nalgebra::Vector3;
pub use planet::{Atmosphere, Planet, EARTH_FISHER_1960, EARTH_SMITHSONIAN, EARTH_SPHERICAL};
use state::{PrimaryState, State};
pub use vehicle::Vehicle;

use transformations::Transformations;

pub struct Simulation {
    state: PrimaryState,
    vehicle: Vehicle,
    planet: Planet,
    transformations: Transformations,
    integrator: Integrator,
    stepsize: f64,
}

impl Simulation {
    pub fn new(vehicle: Vehicle, planet: Planet, stepsize: f64, launch: [f64; 3]) -> Self {
        Simulation {
            state: PrimaryState::default(),
            vehicle,
            planet,
            transformations: Transformations::new(launch),
            integrator: Integrator::RK4,
            stepsize,
        }
    }

    fn system(&self, state: &PrimaryState) -> State {
        let attitude = self.vehicle.steer(state.time);

        let ib = self
            .transformations
            .inertial_to_body(attitude.x, attitude.y, attitude.z);
        let state = state.add_attitude(attitude, ib);

        let pressure = self.planet.pressure(state.position);
        let alpha = self.planet.alpha(ib.transform_vector(&state.velocity));
        let mach = self.planet.mach_number(state.position, state.velocity);
        let dynamic_pressure = self.planet.dynamic_pressure(state.position, state.velocity);
        let state = state.add_env(pressure, alpha, mach, dynamic_pressure);

        let mut thrust = self.vehicle.thrust(pressure);
        let aero = self.vehicle.aero(alpha, mach, dynamic_pressure);

        thrust = self.vehicle.auto_throttle(thrust, aero);

        let gravity = self.planet.gravity(state.position);

        state.add_differentials(
            state.body_to_inertial.transform_vector(&(thrust + aero)) + gravity,
            0.,
        )
    }

    pub fn step(&mut self) -> &PrimaryState {
        self.state = self
            .integrator
            .step(|state| self.system(state), &self.state, self.stepsize);
        &self.state
    }
}

impl Simulation {
    pub fn init_inertial(&mut self, position: Vector3<f64>, velocity: Vector3<f64>) {
        self.state.position = position;
        self.state.velocity = velocity;
    }
}
