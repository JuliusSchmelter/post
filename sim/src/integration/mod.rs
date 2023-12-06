// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 14.11.23
// Last modified by Tibor Völcker on 06.12.23
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

mod runge_kutta;

pub use runge_kutta::RK4;

use crate::System;

pub trait Integrator {
    // Function to integrate the system one time step.
    // It should set the new state and time.
    fn step<const D: usize>(&self, system: &mut impl System<D>, stepsize: f64);
}
