// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 12.11.23
// Last modified by Tibor Völcker on 06.12.23
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

// allow dead code for now, as it's still WIP
#![allow(dead_code)]

use nalgebra::{SVector, Vector6};

mod atmosphere;
pub mod integration;
mod planet;
mod utils;
mod vehicle;

pub use atmosphere::Atmosphere;
pub use integration::Integrator;
pub use planet::Planet;
pub use vehicle::Vehicle;

pub trait System<const D: usize> {
    fn system(&self, time: f64, state: &SVector<f64, D>) -> SVector<f64, D>;

    fn get_state(&self) -> SVector<f64, D>;

    fn get_time(&self) -> f64;

    fn set_state(&mut self, state: SVector<f64, D>);

    fn set_time(&mut self, time: f64);
}

pub struct TranslationalEquations {
    pub time: f64,
    pub vehicle: Vehicle,
    pub planet: Planet,
}

impl TranslationalEquations {
    pub fn new(vehicle: Vehicle, planet: Planet) -> Self {
        TranslationalEquations {
            time: 0.,
            vehicle,
            planet,
        }
    }
}

impl System<6> for TranslationalEquations {
    fn get_time(&self) -> f64 {
        self.time
    }

    fn get_state(&self) -> Vector6<f64> {
        return Vector6::from_row_slice(
            &[
                self.vehicle.position.as_slice(),
                self.vehicle.velocity.as_slice(),
            ]
            .concat(),
        );
    }
    fn set_state(&mut self, state: Vector6<f64>) {
        self.vehicle.position = state.fixed_rows::<3>(0).into();
        self.vehicle.velocity = state.fixed_rows::<3>(3).into();
    }

    fn set_time(&mut self, time: f64) {
        self.time = time;
    }

    fn system(&self, _time: f64, state: &Vector6<f64>) -> Vector6<f64> {
        // r_dot_I = V_I
        // V_dot_I = [IB]^-1 [A_TB + A_AB] + G_I

        let gravity = self.planet.gravity(state.fixed_rows::<3>(0).into());

        return Vector6::from_row_slice(
            &[state.fixed_rows::<3>(3).as_slice(), gravity.as_slice()].concat(),
        );
    }
}
