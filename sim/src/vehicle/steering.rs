// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 06.12.23
// Last modified by Tibor Völcker on 23.02.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::Vector4;

use crate::atmosphere::State;

#[derive(Clone)]
pub enum Steering {
    // Rate,
    Angular(Angular),
}

#[derive(Clone)]
pub enum Angular {
    Polynomials(Vector4<f64>),
    // Tables,
    // LinearEquations,
    // ClosedLoop,
}

impl Steering {
    pub fn update(&self, state: &State) -> f64 {
        match self {
            Steering::Angular(steering_type) => match steering_type {
                Angular::Polynomials(coeffs) => {
                    // See [1] p. V-22
                    Vector4::from_iterator((0..4).map(|i| state.time.powi(i)))
                        .component_mul(coeffs)
                        .sum()
                }
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::vector;

    use super::*;

    #[test]
    fn angular_polynomials() {
        let steering = Steering::Angular(Angular::Polynomials(vector![4., 3., 2., 1.]));
        let state = State {
            time: 2.,
            ..Default::default()
        };

        assert_eq!(
            steering.update(&state),
            4. + 3. * 2. + 2. * 2_f64.powi(2) + 1. * 2_f64.powi(3)
        )
    }
}
