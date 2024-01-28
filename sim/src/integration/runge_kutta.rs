// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 12.11.23
// Last modified by Tibor Völcker on 28.01.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::{matrix, vector, SMatrix, SVector};

pub struct RungeKutta<const R: usize> {
    a: SMatrix<f64, R, R>,
    b: SVector<f64, R>,
    c: SVector<f64, R>,
}

impl<const R: usize> RungeKutta<R> {
    pub fn step<const D: usize>(
        &self,
        f: impl Fn(f64, &SVector<f64, D>) -> SVector<f64, D>,
        x_n: f64,
        y_n: SVector<f64, D>,
        h: f64,
    ) -> SVector<f64, D> {
        let mut k = SMatrix::<f64, D, R>::zeros();

        for i in 0..R {
            // See [1] p. VI-12
            // k_i = h*f(x_n + c_i*h, y_n + SUM[a_ij * k_j])
            let ki = h * f(
                x_n + self.c[i] * h,
                &(y_n
                    + (0..R)
                        .map(|j| self.a[(i, j)] * k.column(j))
                        .sum::<SVector<f64, D>>()),
            );
            k.set_column(i, &ki);
        }

        // See [1] p. VI-12
        // y_n+1 = y_n + SUM[b_i * k_i]
        // This could be done in one loop, but would be less readable
        y_n + (0..R)
            .map(|i| self.b[i] * k.column(i))
            .sum::<SVector<f64, D>>()
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
    use utils::assert_lt;

    pub struct Example {
        x: f64,
        y: Vector2<f64>,
    }

    fn initial() -> (f64, Vector2<f64>) {
        (0., Vector2::new(-0.5, 0.5))
    }

    fn solution(x: f64) -> Vector2<f64> {
        // y_1 = 1/3*x^3 + x^2 + x - 0.5e^x
        // y_2 = x^2 + 2x + 1 - 0.5e^x
        vector![
            1. / 3. * x.powi(3) + x.powi(2) + x - 0.5 * x.exp(),
            x.powi(2) + 2. * x + 1. - 0.5 * x.exp()
        ]
    }

    fn system(x: f64, y: &Vector2<f64>) -> Vector2<f64> {
        // y_0' = y_1
        // y_1' = y_1 - x^2 + 1
        vector![y[1], (y[1] - x.powi(2) + 1.)]
    }

    #[test]
    fn rk4_integrate() {
        let (mut x, mut y) = initial();
        let h = 0.5;

        let mut avg_err = 0.;
        while x <= 4. {
            y = RK4.step(system, x, y, h);
            x += h;

            let err = (solution(x) - y).abs();
            avg_err += err.norm();

            println!("Time: {:.1}", x);
            println!("---------");
            println!(
                "x: Solution={:5.2}, State={:5.2}, Error={:.1e}",
                solution(x)[0],
                y[0],
                err[0]
            );
            println!(
                "y: Solution={:5.2}, State={:5.2}, Error={:.1e}\n",
                solution(x)[1],
                y[1],
                err[1]
            );
        }
        avg_err /= 9.;

        println!("Avg. Error={avg_err:.2e}");

        assert_lt!(avg_err, 5e-2);
    }

    #[test]
    fn rk4_integrate_smaller_stepsize() {
        let (mut x, mut y) = initial();
        let h = 0.1;

        let mut avg_err = 0.;
        while x <= 4. {
            y = RK4.step(system, x, y, h);
            x += h;

            let err = (solution(x) - y).abs();
            avg_err += err.norm();
        }
        avg_err /= 41.;

        assert_lt!(avg_err, 5e-5);
    }
}
