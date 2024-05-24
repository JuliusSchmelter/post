// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 14.11.23
// Last modified by Tibor Völcker on 24.05.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

//! Handles the integration.
//!
//! It defines the enum `Integrator` where the integration method can be
//! selected, and which handles the conversion from vectors to the [`State`]
//! struct.

mod runge_kutta;

use crate::state::State;
use nalgebra::{vector, SVector, Vector2};

/// Represents a generic interpolator. Used to select the desired integration
/// method.
#[derive(Debug, Clone)]
pub enum Integrator {
    /// Runge-Kutta 4th order. See [`runge_kutta`].
    RK4,
}

impl Integrator {
    /// Does one integration step. It converts the function `func` from
    /// `impl Fn(State) -> State` to `impl Fn(Vector2, Vector7) -> Vector7`,
    /// which can then be integrated by the underlying integrators. The
    /// `Vector2` is the two time states, the `Vector7` is the primary state.
    /// See [`State::to_primary_vec`] for more information.
    ///
    /// Then, it calls the underlying integration method.
    pub(crate) fn step(
        &self,
        func: impl Fn(State) -> State,
        state: &State,
        stepsize: f64,
    ) -> State {
        // convert states to vectors and back for the translational equations
        let converted_func = |t: Vector2<f64>, s: SVector<f64, 7>| {
            func(State::from_vec(t, s)).to_differentials_vector()
        };

        match self {
            Integrator::RK4 => {
                let (time_vec, state_vec) = runge_kutta::RK4.step(
                    // convert states to vectors and back
                    converted_func,
                    vector![state.time, state.time_since_event],
                    state.to_primary_vec(),
                    stepsize,
                );

                // Run translational equations again for full state output
                func(State::from_vec(time_vec, state_vec))
            }
        }
    }
}
