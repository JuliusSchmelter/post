// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 12.11.23
// Last modified by Tibor Völcker on 16.12.23
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

// allow dead code for now, as it's still WIP
#![allow(dead_code)]

use nalgebra::Vector6;

mod atmosphere;
pub mod integration;
mod planet;
mod utils;
pub mod vehicle;

pub use atmosphere::Atmosphere;
pub use integration::Integrator;
pub use planet::Planet;
pub use vehicle::Vehicle;

pub struct Simulation {
    pub time: f64,
    pub vehicle: Vehicle,
    pub planet: Planet,
    integrator: Integrator,
    stepsize: f64,
}

impl Simulation {
    pub fn new(vehicle: Vehicle, planet: Planet, stepsize: f64) -> Self {
        Simulation {
            time: 0.,
            vehicle,
            planet,
            integrator: Integrator::RK4,
            stepsize,
        }
    }

    fn system(&self, _time: f64, state: &Vector6<f64>) -> Vector6<f64> {
        // r_dot_I = V_I
        // V_dot_I = [IB]^-1 [A_TB + A_AB] + G_I

        let gravity = self.planet.gravity(state.fixed_rows::<3>(0).into());

        return Vector6::from_row_slice(
            &[state.fixed_rows::<3>(3).as_slice(), gravity.as_slice()].concat(),
        );
    }

    pub fn step(&mut self) {
        // assemble old state
        let mut state = Vector6::from_row_slice(
            &[
                self.vehicle.position.as_slice(),
                self.vehicle.velocity.as_slice(),
            ]
            .concat(),
        );

        // integrate system
        state = self.integrator.step(
            |time, state| self.system(time, state),
            self.time,
            state,
            self.stepsize,
        );

        // set new state
        self.vehicle.position = state.fixed_rows::<3>(0).into();
        self.vehicle.velocity = state.fixed_rows::<3>(3).into();
        self.time += self.stepsize;
    }
}
