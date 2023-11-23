// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 12.11.23
// Last modified by Tibor Völcker on 18.11.23
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::{SVector, Vector3, Vector6};

pub mod integration;
pub mod planet;

use planet::Planet;

pub trait System<const D: usize> {
    fn system(&self, time: f32, state: &SVector<f32, D>) -> SVector<f32, D>;

    fn get_state(&self) -> &SVector<f32, D>;

    fn get_time(&self) -> f32;

    fn set_state(&mut self, state: SVector<f32, D>);

    fn set_time(&mut self, time: f32);
}

pub struct TranslationalEquations<P: Planet> {
    time: f32,
    // state = [position, velocity]
    state: Vector6<f32>,
    planet: P,
}

impl<P: Planet> TranslationalEquations<P> {
    pub fn new(time: f32, state: Vector6<f32>, planet: P) -> Self {
        return TranslationalEquations {
            time,
            state,
            planet,
        };
    }

    pub fn position(&self) -> Vector3<f32> {
        self.state.fixed_rows::<3>(0).into()
    }

    pub fn velocity(&self) -> Vector3<f32> {
        self.state.fixed_rows::<3>(3).into()
    }
}

impl<P: Planet> System<6> for TranslationalEquations<P> {
    fn get_time(&self) -> f32 {
        return self.time;
    }

    fn get_state(&self) -> &Vector6<f32> {
        return &self.state;
    }
    fn set_state(&mut self, state: Vector6<f32>) {
        self.state = state;
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
    use std::f32::consts::PI;

    use nalgebra::vector;

    use super::*;
    use crate::sim::integration::runge_kutta::RK4;
    use crate::sim::integration::Integrator;
    use crate::sim::planet::SPHERICAL_EARTH;

    #[test]
    fn circular_orbit() {
        let r: f32 = 7000e3;
        // v^2 = mu / r
        let v = f32::sqrt(SPHERICAL_EARTH.mu / r);
        // T = 2 PI * sqrt(r^3 / mu)
        let period = 2. * PI * f32::sqrt(r.powi(3) / SPHERICAL_EARTH.mu);

        let mut system =
            TranslationalEquations::new(0., vector![r, 0., 0., 0., v, 0.], SPHERICAL_EARTH);

        while system.time < period {
            RK4.step(&mut system, 10.);
            assert!(
                (system.position().norm() - 7000e3 < 10e3),
                "Distance from planet should always be 7000 km but is: {:.0} km",
                system.position().norm() / 1000.
            );
            assert_eq!(
                system.position()[2],
                0.,
                "Third entry in position should always be 0, but is: {} km",
                system.position()[2]
            );
            assert_eq!(
                system.velocity()[2],
                0.,
                "Third entry in velocity should always be 0, but is: {} km",
                system.velocity()[2]
            );
        }

        assert!(
            (system.position()[0] - 7000e3).abs() < 10e3,
            "First entry in position should be roughly 7000 km after full orbit, but is: {:.0} km",
            system.position()[0] / 1000.
        );
        assert!(
            system.position()[1].abs() < 50e3,
            "Second entry in position should be roughly 0 km after full orbit, but is: {:.0} km",
            system.position()[1] / 1000.
        );
        assert!(
            system.velocity()[0].abs() < 10e3,
            "Second entry in position should be roughly 0 m/s after full orbit, but is: {:.0} km",
            system.velocity()[0]
        );
        assert!(
            (system.velocity()[1] - v).abs() < 10.,
            "First entry in position should be roughly {:.0} m/s after full orbit, but is: {:.0} km",
            v,
            system.position()[0]
        );
    }
}
