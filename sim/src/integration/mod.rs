// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 14.11.23
// Last modified by Tibor Völcker on 28.01.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use crate::state::{PrimaryState, State};

mod runge_kutta;

pub enum Integrator {
    RK4,
}

impl Integrator {
    pub(crate) fn step(
        &self,
        func: impl Fn(&PrimaryState) -> State,
        state: &PrimaryState,
        stepsize: f64,
    ) -> PrimaryState {
        match self {
            Integrator::RK4 => (
                state.time + stepsize,
                runge_kutta::RK4.step(
                    |time, &state| func(&(time, state).into()).differentials(),
                    state.time,
                    state.into(),
                    stepsize,
                ),
            )
                .into(),
        }
    }
}
