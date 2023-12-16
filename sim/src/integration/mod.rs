// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 14.11.23
// Last modified by Tibor Völcker on 16.12.23
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::SVector;

mod runge_kutta;

pub enum Integrator {
    RK4,
}

type State<const D: usize> = SVector<f64, D>;

impl Integrator {
    pub fn step<const D: usize>(
        &self,
        func: impl Fn(f64, &State<D>) -> State<D>,
        time: f64,
        state: State<D>,
        stepsize: f64,
    ) -> State<D> {
        match self {
            Integrator::RK4 => runge_kutta::RK4.step(func, time, state, stepsize),
        }
    }
}
