// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 12.11.23
// Last modified by Tibor Völcker on 22.05.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::{matrix, vector, SMatrix, SVector};

pub struct RungeKutta<const R: usize> {
    a: SMatrix<f64, R, R>,
    b: SVector<f64, R>,
    c: SVector<f64, R>,
}

impl<const R: usize> RungeKutta<R> {
    pub fn step<const D_X: usize, const D_Y: usize>(
        &self,
        f: impl Fn(SVector<f64, D_X>, SVector<f64, D_Y>) -> SVector<f64, D_Y>,
        x_n: SVector<f64, D_X>,
        y_n: SVector<f64, D_Y>,
        h: f64,
    ) -> (SVector<f64, D_X>, SVector<f64, D_Y>) {
        let mut k = SMatrix::<f64, D_Y, R>::zeros();

        for i in 0..R {
            // See [1] p. VI-12
            // k_i = h*f(x_n + c_i*h, y_n + SUM[a_ij * k_j])
            let ki = h * f(
                x_n.add_scalar(self.c[i] * h),
                y_n + (0..R)
                    .map(|j| self.a[(i, j)] * k.column(j))
                    .sum::<SVector<f64, D_Y>>(),
            );
            k.set_column(i, &ki);
        }

        // See [1] p. VI-12
        // y_n+1 = y_n + SUM[b_i * k_i]
        // This could be done in one loop, but would be less readable
        (
            x_n.add_scalar(h),
            y_n + (0..R)
                .map(|i| self.b[i] * k.column(i))
                .sum::<SVector<f64, D_Y>>(),
        )
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
    use nalgebra::{Vector1, Vector2};

    fn initial() -> (Vector1<f64>, Vector2<f64>) {
        (Vector1::new(0.), Vector2::new(-0.5, 0.5))
    }

    fn solution(x: Vector1<f64>) -> Vector2<f64> {
        // y_1 = 1/3*x^3 + x^2 + x - 0.5e^x
        // y_2 = x^2 + 2x + 1 - 0.5e^x
        let x = x.to_scalar();
        vector![
            1. / 3. * x.powi(3) + x.powi(2) + x - 0.5 * x.exp(),
            x.powi(2) + 2. * x + 1. - 0.5 * x.exp()
        ]
    }

    fn system(x: Vector1<f64>, y: Vector2<f64>) -> Vector2<f64> {
        // y_0' = y_1
        // y_1' = y_1 - x^2 + 1
        let x = x.to_scalar();
        vector![y[1], (y[1] - x.powi(2) + 1.)]
    }

    #[test]
    /// Tests the Runge Kutta 4th order integrator
    fn rk4_integrate() {
        const END: Vector1<f64> = Vector1::new(4.);
        const H: f64 = 0.5;
        const EPSILON: f64 = 2e-2;

        let (mut x, mut y) = initial();

        let mut avg_err = 0.;
        while x <= END {
            (x, y) = RK4.step(system, x, y, H);

            let err = (solution(x) - y).abs();
            avg_err += err.norm();

            println!(
                "Time: {:.1}\n\
                ─────────     y_1,    y_2\n\
                Solution: [{:6.2}, {:6.2}]\n\
                State:    [{:6.2}, {:6.2}]\n\
                Error:    [{:6.1e}, {:6.1e}]\n",
                x,
                solution(x)[0],
                solution(x)[1],
                y[0],
                y[1],
                err[0],
                y[1],
            );
        }
        avg_err /= END.to_scalar() / H + 1.;

        println!("Avg. Error: {avg_err:.2e}");

        assert!(
            avg_err < EPSILON,
            "Average error is too big!\n  {:.2e} > {:.2e}",
            avg_err,
            EPSILON
        );
    }

    #[test]
    /// Tests if a smaller stepsize will increase integration accuracy
    fn rk4_integrate_smaller_stepsize() {
        const END: Vector1<f64> = Vector1::new(4.);
        const H: f64 = 0.1;
        const EPSILON: f64 = 2e-5;

        let (mut x, mut y) = initial();

        let mut avg_err = 0.;
        while x <= END {
            (x, y) = RK4.step(system, x, y, H);

            let err = (solution(x) - y).abs();
            avg_err += err.norm();
        }
        avg_err /= END.to_scalar() / H + 1.;

        assert!(
            avg_err < EPSILON,
            "Average error is too big!\n  {:.2e} > {:.2e}",
            avg_err,
            EPSILON
        );
    }
}
