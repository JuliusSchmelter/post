// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 06.01.24
// Last modified by Tibor Völcker on 24.05.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)

//! Defines function for linear interpolation.

/// Helper function to retrieve the indexes of the value below and above the
/// passed `val`. If `val` is bigger or smaller than all values in `val_arr`,
/// the two last or two first indexes are returned.
///
/// __Attention:__ The function assumes that `val_arr` is sorted and has at
/// least the length of 2. This is not checked for performance reasons, but
/// should be given by the overlying table implementation.
fn get_idx(val_arr: &[f64], val: f64) -> (usize, usize) {
    // Get index of upper base (index of closes bigger number)
    let idx1 = {
        let mut idx1 = val_arr.partition_point(|i| i < &val);
        if idx1 == val_arr.len() {
            // No bigger number. Use the last two as bases.
            idx1 -= 1;
        } else if idx1 == 0 {
            // All numbers are bigger. Use the first two as bases.
            idx1 = 1;
        }
        idx1
    };

    (idx1 - 1, idx1)
}

/// Trilinear interpolation. Interpolates `data` with `x`, `y` and `z`, as well
/// as the arrays to interpolate: `x_arr`, `y_arr` and `z_arr`.
///
/// Internally calls [`bilinear_interpolate`] twice and interpolates its
/// results.
///
/// __Attention:__ The function assumes that `x_arr`, `y_arr` and `z_arr` are
/// sorted and the lengths match the corresponding array in `data`. This is not
/// checked for performance reasons, but should be given by the overlying table
/// implementation.
pub fn trilinear_interpolate(
    x_arr: &[f64],
    x: f64,
    y_arr: &[f64],
    y: f64,
    z_arr: &[f64],
    z: f64,
    data: &[Box<[Box<[f64]>]>],
) -> f64 {
    // Assumptions:
    //   1. `x_arr`, `y_arr` and `z_arr` are sorted.
    //   2. Lengths of `x_arr`, `y_arr` and `z_arr` correspond to lengths of data.

    if data.is_empty() {
        // No data cannot be interpolated.
        return f64::NAN;
    }
    if data.len() == 1 {
        // Interpolate single data point with a straight line.
        return bilinear_interpolate(y_arr, y, z_arr, z, &data[0]);
    }

    let (idx0, idx1) = get_idx(x_arr, x);

    let x0 = x_arr[idx0];
    let x1 = x_arr[idx1];
    let y0 = bilinear_interpolate(y_arr, y, z_arr, z, &data[idx0]);
    let y1 = bilinear_interpolate(y_arr, y, z_arr, z, &data[idx1]);

    y0 + (x - x0) * (y1 - y0) / (x1 - x0)
}

/// Bilinear interpolation. Interpolates `data` with `x` and `y`, as well as
/// the arrays to interpolate: `x_arr` and `y_arr`.
///
/// Internally calls [`linear_interpolate`] twice and interpolates its
/// results.
///
/// __Attention:__ The function assumes that `x_arr` and `y_arr` are sorted and
/// the lengths match the corresponding array in `data`. This is not checked
/// for performance reasons, but should be given by the overlying table
/// implementation.
pub fn bilinear_interpolate(
    x_arr: &[f64],
    x: f64,
    y_arr: &[f64],
    y: f64,
    data: &[Box<[f64]>],
) -> f64 {
    // Assumptions:
    //   1. `x_arr` and `y_arr` are sorted.
    //   2. Lengths of `x_arr` and `y_arr` correspond to lengths of data.

    if data.is_empty() {
        // No data cannot be interpolated.
        return f64::NAN;
    }
    if data.len() == 1 {
        // Interpolate single data point with a straight line.
        return linear_interpolate(y_arr, y, &data[0]);
    }

    let (idx0, idx1) = get_idx(x_arr, x);

    let x0 = x_arr[idx0];
    let x1 = x_arr[idx1];
    let y0 = linear_interpolate(y_arr, y, &data[idx0]);
    let y1 = linear_interpolate(y_arr, y, &data[idx1]);

    y0 + (x - x0) * (y1 - y0) / (x1 - x0)
}

/// Linear interpolation. Interpolates `data` with `x`, as well as the array to
/// interpolate: `x_arr`.
///
/// __Attention:__ The function assumes that `x_arr` is sorted and the length
/// matches `data`. This is not checked for performance reasons, but should be
/// given by the overlying table implementation.
pub fn linear_interpolate(x_arr: &[f64], x: f64, data: &[f64]) -> f64 {
    // Assumptions:
    //   1. `x_arr` is sorted.
    //   2. Length of `x_arr` correspond to length of data.

    if data.is_empty() {
        // No data cannot be interpolated.
        return f64::NAN;
    }
    if data.len() == 1 {
        // Interpolate single data point with a straight line.
        return data[0];
    }

    let (idx0, idx1) = get_idx(x_arr, x);

    let x0 = x_arr[idx0];
    let x1 = x_arr[idx1];
    let y0 = data[idx0];
    let y1 = data[idx1];

    y0 + (x - x0) * (y1 - y0) / (x1 - x0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty() {
        let x_arr = [];
        let data = [];

        assert!(linear_interpolate(&x_arr, 1.25, &data).is_nan())
    }

    #[test]
    fn one_entry() {
        let x_arr = [0.];
        let data = [1.34];

        assert_eq!(linear_interpolate(&x_arr, 0., &data), 1.34);

        assert_eq!(linear_interpolate(&x_arr, 999., &data), 1.34);
    }

    #[test]
    fn extrapolate_below() {
        let x_arr = [2., 3., 4., 5.];
        let data = [20., 30., 40., 50.];

        // Extrapolate below
        assert_eq!(linear_interpolate(&x_arr, 1., &data), 10.);
        // Extrapolate above
        assert_eq!(linear_interpolate(&x_arr, 6., &data), 60.);
        // Interpolate included data point
        assert_eq!(linear_interpolate(&x_arr, 4., &data), 40.);
        // Interpolate between data points
        assert_eq!(linear_interpolate(&x_arr, 3.5, &data), 35.);
        // Interpolate random data point
        assert_eq!(linear_interpolate(&x_arr, 3.125, &data), 31.25);
    }

    #[test]
    fn empty_2d() {
        let x_arr = [];
        let y_arr = [];
        let data = [[].into()];

        assert!(bilinear_interpolate(&x_arr, 1.25, &y_arr, 3.61, &data).is_nan())
    }

    #[test]
    fn interpolate_2d() {
        let x_arr = [1., 2.];
        let y_arr = [10., 20.];
        let data = [[100., 200.].into(), [300., 400.].into()];

        assert_eq!(bilinear_interpolate(&x_arr, 1.5, &y_arr, 15., &data), 250.)
    }

    #[test]
    fn empty_3d() {
        let x_arr = [];
        let y_arr = [];
        let z_arr = [];
        let data = [[[].into()].into()];

        assert!(trilinear_interpolate(&x_arr, 1.25, &y_arr, 3.61, &z_arr, 9.12, &data).is_nan())
    }

    #[test]
    fn interpolate_3d() {
        let x_arr = [1., 2.];
        let y_arr = [10., 20.];
        let z_arr = [100., 200.];
        let data = [
            [[1000., 2000.].into(), [3000., 4000.].into()].into(),
            [[5000., 6000.].into(), [7000., 8000.].into()].into(),
        ];

        assert_eq!(
            trilinear_interpolate(&x_arr, 1.5, &y_arr, 15., &z_arr, 150., &data),
            4500.
        )
    }
}
