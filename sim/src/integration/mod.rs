// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 14.11.23
// Last modified by Tibor Völcker on 05.03.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

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
                let state_vec = runge_kutta::RK4.step(
                    // convert states to vectors and back
                    |time, &state| func((time, state).into()).differentials(),
                    state.time,
                    state.to_primary_vec(),
                    stepsize,
                );
                let primary_state = (state.time + stepsize, state_vec).into();
                func(primary_state)
            }
        }
    }
}
