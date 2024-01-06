// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 12.11.23
// Last modified by Tibor Völcker on 06.01.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::{matrix, vector, SMatrix, SVector};

use super::State;

pub struct RungeKutta<const R: usize> {
    a: SMatrix<f64, R, R>,
    b: SVector<f64, R>,
    c: SVector<f64, R>,
}

impl<const R: usize> RungeKutta<R> {
    pub fn step<const D: usize>(
        &self,
        func: impl Fn(f64, &State<D>) -> State<D>,
        time: f64,
        state: State<D>,
        stepsize: f64,
    ) -> State<D> {
        let mut k = SMatrix::<f64, D, R>::zeros();

        for i in 0..R {
            // See [1] p. VI-12
            // k_i = h*f(x_n + c_i*h, y_n + SUM[a_ij * k_j])
            let ki = stepsize
                * func(
                    time + self.c[i] * stepsize,
                    &(state
                        + (0..R)
                            .map(|j| self.a[(i, j)] * k.column(j))
                            .sum::<State<D>>()),
                );
            k.set_column(i, &ki);
        }

        // See [1] p. VI-12
        // y_n+1 = y_n + SUM[b_i * k_i]
        // This could be done in one loop, but would be less readable
        state + (0..R).map(|i| self.b[i] * k.column(i)).sum::<State<D>>()
    }
}

pub const RK4: RungeKutta<4> = RungeKutta {
    a: matrix![0., 0., 0., 0.; 0.5, 0., 0., 0.;
    0., 0.5, 0., 0.;
    0., 0., 1., 0.],
    b: vector![1. / 6., 1. / 3., 1. / 3., 1. / 6.],
    c: vector![0., 0.5, 0.5, 1.],
};

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector2;
    use test_utils::assert_lt;

    pub struct Example {
        time: f64,
        // state = [position, velocity]
        state: Vector2<f64>,
    }

    fn initial() -> (f64, Vector2<f64>) {
        (0., Vector2::new(-0.5, 0.5))
    }

    fn solution(time: f64) -> Vector2<f64> {
        // x = 1/3*t^3 + t^2 + t - 0.5e^t
        // y = t^2 + 2t + 1 - 0.5e^t
        vector![
            1. / 3. * time.powi(3) + time.powi(2) + time - 0.5 * time.exp(),
            time.powi(2) + 2. * time + 1. - 0.5 * time.exp()
        ]
    }

    fn system(time: f64, state: &Vector2<f64>) -> Vector2<f64> {
        // x' = y
        // y' = y - t^2 + 1
        vector![state.y, (state.y - time.powi(2) + 1.)]
    }

    #[test]
    fn rk4_integrate() {
        let (mut time, mut state) = initial();
        let stepsize = 0.5;

        let mut avg_err = 0.;
        while time <= 4. {
            state = RK4.step(system, time, state, stepsize);
            time += stepsize;

            let err = (solution(time) - state).abs();
            avg_err += err.norm();

            println!("Time: {:.1}", time);
            println!("---------");
            println!(
                "x: Solution={:5.2}, State={:5.2}, Error={:.1e}",
                solution(time)[0],
                state[0],
                err[0]
            );
            println!(
                "y: Solution={:5.2}, State={:5.2}, Error={:.1e}\n",
                solution(time)[1],
                state[1],
                err[1]
            );
        }
        avg_err /= 9.;

        println!("Avg. Error={avg_err:.2e}");

        assert_lt!(avg_err, 5e-2);
    }

    #[test]
    fn rk4_integrate_smaller_stepsize() {
        let (mut time, mut state) = initial();
        let stepsize = 0.1;

        let mut avg_err = 0.;
        while time <= 4. {
            state = RK4.step(system, time, state, stepsize);
            time += stepsize;

            let err = (solution(time) - state).abs();
            avg_err += err.norm();
        }
        avg_err /= 41.;

        assert_lt!(avg_err, 5e-5);
    }
}
