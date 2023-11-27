// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 12.11.23
// Last modified by Tibor Völcker on 27.11.23
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::{matrix, vector, SMatrix, SVector};

use super::Integrator;

pub struct RungeKutta<const D: usize> {
    a: SMatrix<f32, D, D>,
    b: SVector<f32, D>,
    c: SVector<f32, D>,
}

impl<const D: usize> RungeKutta<D> {
    pub const fn new(a: SMatrix<f32, D, D>, b: SVector<f32, D>, c: SVector<f32, D>) -> Self {
        Self { a, b, c }
    }
}

impl<const D: usize> Integrator for RungeKutta<D> {
    fn step<const R: usize>(&self, system: &mut impl crate::sim::System<R>, stepsize: f32) {
        let mut k = SMatrix::<f32, R, D>::zeros();

        for i in 0..D {
            // See [1] p. VI-12
            // k_i = h*f(x_n + c_i*h, y_n + SUM[a_ij * k_j])
            let ki = stepsize
                * system.system(
                    system.get_time() + self.c[i] * stepsize,
                    &(system.get_state()
                        + (0..D)
                            .map(|j| self.a[(i, j)] * k.column(j))
                            .sum::<SVector<f32, R>>()),
                );
            k.set_column(i, &ki);
        }

        // See [1] p. VI-12
        // y_n+1 = y_n + SUM[b_i * k_i]
        // This could be done in one loop, but would be less readable
        let new_state = system.get_state().clone()
            + (0..D)
                .map(|i| self.b[i] * k.column(i))
                .sum::<SVector<f32, R>>();

        system.set_state(new_state);
        system.set_time(system.get_time() + stepsize);
    }
}

pub const RK4: RungeKutta<4> = RungeKutta::new(
    matrix![0., 0., 0., 0.; 0.5, 0., 0., 0.;
    0., 0.5, 0., 0.;
    0., 0., 1., 0.],
    vector![1. / 6., 1. / 3., 1. / 3., 1. / 6.],
    vector![0., 0.5, 0.5, 1.],
);

#[cfg(test)]
mod tests {
    use nalgebra::Vector2;

    use crate::sim::System;

    use super::*;

    pub struct Example {
        time: f32,
        // state = [position, velocity]
        state: Vector2<f32>,
    }

    impl Example {
        pub fn new() -> Self {
            return Self {
                time: 0.,
                state: Vector2::new(-0.5, 0.5),
            };
        }

        fn solution(&self) -> Vector2<f32> {
            // x = 1/3*t^3 + t^2 + t - 0.5e^t
            // y = t^2 + 2t + 1 - 0.5e^t
            return vector![
                1. / 3. * self.time.powi(3) + self.time.powi(2) + self.time - 0.5 * self.time.exp(),
                self.time.powi(2) + 2. * self.time + 1. - 0.5 * self.time.exp()
            ];
        }
    }

    impl System<2> for Example {
        fn get_time(&self) -> f32 {
            return self.time;
        }

        fn get_state(&self) -> Vector2<f32> {
            return self.state;
        }
        fn set_state(&mut self, state: Vector2<f32>) {
            self.state = state;
        }

        fn set_time(&mut self, time: f32) {
            self.time = time;
        }

        fn system(&self, time: f32, state: &Vector2<f32>) -> Vector2<f32> {
            // x' = y
            // y' = y - t^2 + 1
            return vector![state.y, (state.y - time.powi(2) + 1.)];
        }
    }

    #[test]
    fn rk4_integrate() {
        let mut example = Example::new();

        let mut avg_err = 0.;
        while example.time <= 4. {
            RK4.step(&mut example, 0.5);
            let err = (example.solution() - example.state).abs();
            avg_err += err.norm();

            println!("Time: {:.1}", example.time);
            println!("---------");
            println!(
                "x: Solution={:5.2}, State={:5.2}, Error={:.1e}",
                example.solution()[0],
                example.state[0],
                err[0]
            );
            println!(
                "y: Solution={:5.2}, State={:5.2}, Error={:.1e}\n",
                example.solution()[1],
                example.state[1],
                err[1]
            );
        }
        avg_err = avg_err / 9.;

        println!("Avg. Error={avg_err:.2e}");

        assert!(
            avg_err < 5e-2,
            "Average error is too big ({avg_err:.2e} > 5e-2)"
        )
    }

    #[test]
    fn rk4_integrate_smaller_stepsize() {
        let mut example = Example::new();

        let mut avg_err = 0.;
        while example.time <= 4. {
            RK4.step(&mut example, 0.1);
            let err = (example.solution() - example.state).abs();
            avg_err += err.norm();
        }
        avg_err = avg_err / 41.;

        assert!(
            avg_err < 5e-5,
            "Average error is too big ({avg_err:.2e} > 5e-5)"
        )
    }
}
