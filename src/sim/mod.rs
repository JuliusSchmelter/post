// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 12.11.23
// Last modified by Tibor Völcker on 28.11.23
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::{SVector, Vector6};

pub mod atmosphere;
pub mod integration;
pub mod planet;
pub mod utils;
pub mod vehicle;

use planet::Planet;
use vehicle::Vehicle;

pub trait System<const D: usize> {
    fn system(&self, time: f32, state: &SVector<f32, D>) -> SVector<f32, D>;

    fn get_state(&self) -> SVector<f32, D>;

    fn get_time(&self) -> f32;

    fn set_state(&mut self, state: SVector<f32, D>);

    fn set_time(&mut self, time: f32);
}

pub struct TranslationalEquations {
    time: f32,
    vehicle: Vehicle,
    planet: Planet,
}

impl TranslationalEquations {
    pub fn new(vehicle: Vehicle, planet: Planet) -> Self {
        return TranslationalEquations {
            time: 0.,
            vehicle,
            planet,
        };
    }
}

impl System<6> for TranslationalEquations {
    fn get_time(&self) -> f32 {
        return self.time;
    }

    fn get_state(&self) -> Vector6<f32> {
        return Vector6::from_row_slice(
            &[
                self.vehicle.position.as_slice(),
                self.vehicle.velocity.as_slice(),
            ]
            .concat(),
        );
    }
    fn set_state(&mut self, state: Vector6<f32>) {
        self.vehicle.position = state.fixed_rows::<3>(0).into();
        self.vehicle.velocity = state.fixed_rows::<3>(3).into();
    }

    fn set_time(&mut self, time: f32) {
        self.time = time;
    }

    fn system(&self, time: f32, state: &Vector6<f32>) -> Vector6<f32> {
        // r_dot_I = V_I
        // V_dot_I = [IB]^-1 [A_TB + A_AB] + G_I

        let gravity = self.planet.gravity(state.fixed_rows::<3>(0).into());

        return Vector6::from_row_slice(
            &[state.fixed_rows::<3>(3).as_slice(), gravity.as_slice()].concat(),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::assert_almost_eq;
    use crate::sim::integration::runge_kutta::RK4;
    use crate::sim::integration::Integrator;
    use nalgebra::vector;
    use std::f32::consts::PI;

    #[test]
    fn circular_orbit() {
        let planet = Planet::earth_spherical(None);
        let r: f32 = 7000e3;
        // v^2 = mu / r
        let v = f32::sqrt(planet.mu() / r);
        // T = 2 PI * sqrt(r^3 / mu)
        let period = 2. * PI * f32::sqrt(r.powi(3) / planet.mu());

        let mut system = TranslationalEquations::new(Vehicle::new(10e3, vec![]), planet);
        system.vehicle.position = vector![r, 0., 0.];
        system.vehicle.velocity = vector![0., v, 0.];

        while system.time < period {
            RK4.step(&mut system, 10.);
            println!(
                "Time: {:.0}\nPosition: {:.0}\nVelocity: {:.0}",
                system.time, system.vehicle.position, system.vehicle.velocity
            );
            assert_almost_eq!(system.vehicle.position.norm(), 7000e3, 10e3);
            assert_eq!(system.vehicle.position[2], 0.);
            assert_eq!(system.vehicle.velocity[2], 0.);
        }

        assert_almost_eq!(system.vehicle.position[0], 7000e3, 10e3);
        assert_almost_eq!(system.vehicle.position[1].abs(), 0., 50e3);
        assert_almost_eq!(system.vehicle.velocity[0].abs(), 0., 10e3);
        assert_almost_eq!(system.vehicle.velocity[1], v, 10.);
    }
}
