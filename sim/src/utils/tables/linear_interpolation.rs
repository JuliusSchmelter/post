// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 06.01.24
// Last modified by Tibor Völcker on 24.05.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)

//! Defines function for linear interpolation.

use super::TableData;

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

/// Linear interpolation.
/// __Attention:__ Assumes that arrays in `arr` are sorted.
pub fn interpolate(arr: &[&[f64]], at: &[f64], data: &TableData) -> f64 {
    assert_eq!(arr.len(), at.len(), "length of `arr` and `at` must match.");

    match data {
        TableData::Table(table) => {
            if table.is_empty() {
                // No data cannot be interpolated.
                return f64::NAN;
            }
            if table.len() == 1 {
                // Interpolate single data point with a straight line.
                return interpolate(&arr[1..], &at[1..], &table[0]);
            }

            let x_arr = arr[0];
            let x = at[0];

            let (idx0, idx1) = get_idx(x_arr, x);

            let x0 = x_arr[idx0];
            let x1 = x_arr[idx1];
            let y0 = interpolate(&arr[1..], &at[1..], &table[idx0]);
            let y1 = interpolate(&arr[1..], &at[1..], &table[idx1]);

            y0 + (x - x0) * (y1 - y0) / (x1 - x0)
        }
        TableData::Values(values) => {
            if values.is_empty() {
                // No data cannot be interpolated.
                return f64::NAN;
            }

            assert!(arr.len() == 1, "dimension mismatch in interpolate");
            assert!(at.len() == 1, "dimension mismatch in interpolate");
            assert!(
                values.len() == arr[0].len(),
                "data length mismatch in interpolate"
            );

            if values.len() == 1 {
                // Interpolate single data point with a straight line.
                return values[0];
            }

            let x_arr = arr[0];
            let x = at[0];

            let (idx0, idx1) = get_idx(x_arr, x);

            let x0 = x_arr[idx0];
            let x1 = x_arr[idx1];
            let y0 = values[idx0];
            let y1 = values[idx1];

            y0 + (x - x0) * (y1 - y0) / (x1 - x0)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty() {
        let x_arr = [];
        let data = TableData::Values([].into());

        assert!(interpolate(&[&x_arr], &[1.25], &data).is_nan())
    }

    #[test]
    fn one_entry() {
        let x_arr = [0.];
        let data = TableData::Values([1.34].into());

        assert_eq!(interpolate(&[&x_arr], &[0.], &data), 1.34);

        assert_eq!(interpolate(&[&x_arr], &[999.], &data), 1.34);
    }

    #[test]
    fn extrapolate_below() {
        let x_arr = [2., 3., 4., 5.];
        let data = TableData::Values([20., 30., 40., 50.].into());

        // Extrapolate below
        assert_eq!(interpolate(&[&x_arr], &[1.], &data), 10.);
        // Extrapolate above
        assert_eq!(interpolate(&[&x_arr], &[6.], &data), 60.);
        // Interpolate included data point
        assert_eq!(interpolate(&[&x_arr], &[4.], &data), 40.);
        // Interpolate between data points
        assert_eq!(interpolate(&[&x_arr], &[3.5], &data), 35.);
        // Interpolate random data point
        assert_eq!(interpolate(&[&x_arr], &[3.125], &data), 31.25);
    }

    #[test]
    fn empty_2d() {
        let x_arr = [];
        let y_arr = [];
        let data = TableData::Table([TableData::Values([].into())].into());

        assert!(interpolate(&[&x_arr, &y_arr], &[1.25, 3.61], &data).is_nan())
    }

    #[test]
    fn interpolate_2d() {
        let x_arr = [1., 2.];
        let y_arr = [10., 20.];
        let data = TableData::Table(
            [
                TableData::Values([100., 200.].into()),
                TableData::Values([300., 400.].into()),
            ]
            .into(),
        );

        assert_eq!(interpolate(&[&x_arr, &y_arr], &[1.5, 15.], &data), 250.)
    }

    #[test]
    fn empty_3d() {
        let x_arr = [];
        let y_arr = [];
        let z_arr = [];
        let data =
            TableData::Table([TableData::Table([TableData::Values([].into())].into())].into());

        assert!(interpolate(&[&x_arr, &y_arr, &z_arr], &[1.25, 3.61, 9.12], &data).is_nan())
    }

    #[test]
    fn interpolate_3d() {
        let x_arr = [1., 2.];
        let y_arr = [10., 20.];
        let z_arr = [100., 200.];
        let data = TableData::Table(
            [
                TableData::Table(
                    [
                        TableData::Values([1000., 2000.].into()),
                        TableData::Values([3000., 4000.].into()),
                    ]
                    .into(),
                ),
                TableData::Table(
                    [
                        TableData::Values([5000., 6000.].into()),
                        TableData::Values([7000., 8000.].into()),
                    ]
                    .into(),
                ),
            ]
            .into(),
        );

        assert_eq!(
            interpolate(&[&x_arr, &y_arr, &z_arr], &[1.5, 15., 150.], &data),
            4500.
        )
    }
    #[test]
    fn interpolate_4d() {
        let a_arr = [0., 1.];
        let x_arr = [1., 2.];
        let y_arr = [10., 20.];
        let z_arr = [100., 200.];
        let data = TableData::Table(
            [
                TableData::Table(
                    [
                        TableData::Table(
                            [
                                TableData::Values([2000., 4000.].into()),
                                TableData::Values([6000., 8000.].into()),
                            ]
                            .into(),
                        ),
                        TableData::Table(
                            [
                                TableData::Values([10000., 12000.].into()),
                                TableData::Values([14000., 16000.].into()),
                            ]
                            .into(),
                        ),
                    ]
                    .into(),
                ),
                TableData::Table(
                    [
                        TableData::Table(
                            [
                                TableData::Values([1000., 2000.].into()),
                                TableData::Values([3000., 4000.].into()),
                            ]
                            .into(),
                        ),
                        TableData::Table(
                            [
                                TableData::Values([5000., 6000.].into()),
                                TableData::Values([7000., 8000.].into()),
                            ]
                            .into(),
                        ),
                    ]
                    .into(),
                ),
            ]
            .into(),
        );

        assert_eq!(
            interpolate(
                &[&a_arr, &x_arr, &y_arr, &z_arr],
                &[0.5, 1.5, 15., 150.],
                &data
            ),
            6750.
        )
    }
}
