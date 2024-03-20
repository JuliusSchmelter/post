// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 14.11.23
// Last modified by Tibor Völcker on 20.03.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::{vector, SVector, Vector2};

use crate::state::State;

mod runge_kutta;

#[derive(Debug, Clone)]
pub enum Integrator {
    RK4,
}

impl Integrator {
    pub(crate) fn step(&self, func: impl Fn(&mut State), state: &State, stepsize: f64) -> State {
        // convert states to vectors and back for the translational equations
        let converted_func = |t: Vector2<f64>, s: SVector<f64, 7>| {
            let mut state = State::from_vec(t, s);
            func(&mut state);
            state.to_differentials_vector()
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

                // convert vector to a state object
                let mut state = State::from_vec(time_vec, state_vec);

                // Run translational equations again for full state output
                func(&mut state);

                state
            }
        }
    }
}
