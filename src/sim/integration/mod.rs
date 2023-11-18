/*
 * Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 14.11.23
 * Last modified by Tibor Völcker on 18.11.23
 * Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)
 */
use super::System;

pub mod runge_kutta;

pub trait Integrator {
    // Function to integrate the system one time step.
    // It should set the new state and time.
    fn step<const D: usize>(&self, system: &mut impl System<D>, stepsize: f32);
}
