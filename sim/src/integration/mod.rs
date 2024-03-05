// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 14.11.23
// Last modified by Tibor Völcker on 05.03.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::vector;

use crate::state::{PrimaryState, State};

mod runge_kutta;

pub enum Integrator {
    RK4,
}

impl Integrator {
    pub(crate) fn step(
        &self,
        func: impl Fn(PrimaryState) -> State,
        state: &State,
        stepsize: f64,
    ) -> State {
        match self {
            Integrator::RK4 => {
                let (time_vec, state_vec) = runge_kutta::RK4.step(
                    // convert states to vectors and back
                    |&t, &s| func(PrimaryState::from_vec(t, s)).differentials(),
                    vector![state.time, state.time_since_event],
                    state.to_primary_vec(),
                    stepsize,
                );
                let primary_state = PrimaryState::from_vec(time_vec, state_vec);
                func(primary_state)
            }
        }
    }
}
