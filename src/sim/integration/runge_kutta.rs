/*
 * Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 12.11.23
 * Last modified by Tibor Völcker on 14.11.23
 * Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)
 */

// Generate Runge Kutta functions given their Butcher-Tableau
macro_rules! generate_runge_kutta {
    ($name:tt, $a:expr, $b:expr, $c:expr) => {
        pub fn $name(x: f32, y: f32, f: fn(f32, f32) -> f32, h: f32) -> f32 {
            let mut k: Vec<f32> = vec![];

            let mut sum = y;
            for i in 0..$a.len() {
                // See [1] p. VI-12
                // k_i = h*f(x_n + c_i*h, y_n + SUM[a_ij * k_j])
                let ki = h * f(
                    x + $c[i] * h,
                    y + (0..i).map(|j| $a[i][j] * k[j]).sum::<f32>(),
                );
                k.push(ki);

                // See [1] p. VI-12
                // y_n+1 = y_n + SUM[b_i * k_i]
                sum += $b[i] * ki;
            }

            return sum;
        }
    };
}

generate_runge_kutta!(
    runge_kutta_4,
    [
        [0., 0., 0., 0.],
        [0.5, 0., 0., 0.],
        [0., 0.5, 0., 0.],
        [0., 0., 1., 0.],
    ],
    [1. / 6., 1. / 3., 1. / 3., 1. / 6.],
    [0., 0.5, 0.5, 1.]
);

#[cfg(test)]
mod tests {
    use std::f32::INFINITY;

    use super::*;

    fn sample_problem(
        integrator: fn(f32, f32, fn(f32, f32) -> f32, f32) -> f32,
        stepsize: f32,
        max_err: f32,
    ) -> f32 {
        let y0 = 0.5;

        // y' = y - x^2 + 1
        fn differential_eq(x: f32, y: f32) -> f32 {
            return y - x.powi(2) + 1.;
        }

        // y = x^2 + 2x + 1 - 0.5e^x
        fn solution(x: f32) -> f32 {
            return x.powi(2) + 2. * x + 1. - 0.5 * x.exp();
        }

        let mut y = y0;
        let mut x = 0.;
        let mut avg_error = 0.;
        while x <= 4. {
            y = integrator(x as f32, y, differential_eq, stepsize);
            x += stepsize;
            let err = (y - solution(x as f32)).abs();
            assert!(
                err < max_err,
                "Error too big at x={x}. y={y:.4} but should be {:.4} (error={err})",
                solution(x as f32)
            );
            avg_error += err;
        }
        avg_error = avg_error / (4. / stepsize + 1.).floor();

        println!("Avg. Error: {avg_error}");
        return avg_error;
    }

    #[test]
    fn rk4_integrate_max_error() {
        sample_problem(runge_kutta_4, 0.5, 0.05);
    }

    #[test]
    fn rk4_integrate_more_precision_with_stepsize() {
        let avg_err = sample_problem(runge_kutta_4, 0.5, INFINITY);
        let avg_err_smaller = sample_problem(runge_kutta_4, 0.01, INFINITY);
        assert!(
            avg_err_smaller < avg_err,
            "Lower stepsize resulted in bigger average error. ({avg_err} vs {avg_err_smaller})."
        )
    }
}
