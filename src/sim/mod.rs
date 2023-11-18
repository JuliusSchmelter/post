/*
 * Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 12.11.23
 * Last modified by Tibor Völcker on 18.11.23
 * Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)
 */
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
