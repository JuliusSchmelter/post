// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 14.12.23
// Last modified by Tibor Völcker on 17.01.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use std::marker::PhantomData;

pub mod cubic_interpolation;
pub mod linear_interpolation;

fn is_sorted<T>(data: &[T]) -> bool
where
    T: PartialOrd,
{
    data.windows(2).all(|w| w[0] < w[1])
}

pub trait Interpolator {}

pub struct Table<T, I: Interpolator> {
    data: Box<[T]>,
    x: Box<[f64]>,
    interpolator: PhantomData<I>,
}

type Table1D<I> = Table<f64, I>;
type Table2D<I> = Table<Table1D<I>, I>;
type Table3D<I> = Table<Table2D<I>, I>;

// TODO: Table fail if length is smaller than 2 or x and data don't have the same length

impl<I: Interpolator> Table1D<I> {
    pub fn new<const X: usize>(x: [f64; X], data: [f64; X]) -> Self {
        if !is_sorted(&x) {
            panic!("Data must be sorted by indexing value")
        }
        Self {
            x: Box::new(x),
            data: Box::new(data),
            interpolator: PhantomData,
        }
    }

    pub fn zeros() -> Self {
        Self {
            // TODO: This is very ugly
            x: Box::new([0., 1.]),
            data: Box::new([0., 0.]),
            interpolator: PhantomData,
        }
    }
}

impl<I: Interpolator> Table2D<I> {
    pub fn new<const X: usize, const Y: usize>(
        x: [f64; X],
        y: [f64; Y],
        data: [[f64; Y]; X],
    ) -> Self {
        if !is_sorted(&x) {
            panic!("Data must be sorted by indexing value")
        }
        Self {
            x: Box::new(x),
            data: Box::new(data.map(|row| Table1D::new(y, row))),
            interpolator: PhantomData,
        }
    }

    pub fn zeros() -> Self {
        Self {
            x: Box::new([0., 1.]),
            data: Box::new([Table1D::zeros(), Table1D::zeros()]),
            interpolator: PhantomData,
        }
    }
}

impl<I: Interpolator> Table3D<I> {
    pub fn new<const X: usize, const Y: usize, const Z: usize>(
        x: [f64; X],
        y: [f64; Y],
        z: [f64; Z],
        data: [[[f64; Z]; Y]; X],
    ) -> Self {
        if !is_sorted(&x) {
            panic!("Data must be sorted by indexing value")
        }
        Self {
            x: Box::new(x),
            data: Box::new(data.map(|row| Table2D::new(y, z, row))),
            interpolator: PhantomData,
        }
    }

    pub fn zeros() -> Self {
        Self {
            x: Box::new([0., 1.]),
            data: Box::new([Table2D::zeros(), Table2D::zeros()]),
            interpolator: PhantomData,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::linear_interpolation::*;
    use crate::assert_almost_eq;

    #[test]
    fn drag() {
        let table = Table2D::new(
            [-20., -4., 0., 5., 10., 20., 30.],
            [0., 0.2, 0.6, 0.8, 0.9, 1.3, 1.5, 2., 2.48, 3., 3.9, 40.],
            [
                [
                    0.024, 0.024, 0.026, 0.028, 0.035, 0.93, 0.122, 0.116, 0.1, 0.092, 0.082, 0.03,
                ],
                [
                    0.024, 0.024, 0.026, 0.028, 0.035, 0.93, 0.122, 0.116, 0.1, 0.092, 0.082, 0.03,
                ],
                [
                    0.026, 0.026, 0.026, 0.024, 0.036, 0.092, 0.118, 0.106, 0.091, 0.082, 0.074,
                    0.022,
                ],
                [
                    0.042, 0.042, 0.04, 0.042, 0.076, 0.124, 0.142, 0.124, 0.098, 0.088, 0.079,
                    0.033,
                ],
                [
                    0.076, 0.076, 0.08, 0.1, 0.13, 0.194, 0.192, 0.165, 0.127, 0.114, 0.095, 0.057,
                ],
                [
                    0.36, 0.36, 0.362, 0.44, 0.41, 0.39, 0.36, 0.32, 0.242, 0.224, 0.216, 0.238,
                ],
                [
                    0.36, 0.36, 0.36, 0.44, 0.41, 0.39, 0.36, 0.32, 0.44, 0.418, 0.4, 0.3,
                ],
            ],
        );
        assert_almost_eq!(table.at(4.72845235e0, 2.47287807e1), 5.19996124e-2, 1e-10);
        assert_almost_eq!(table.at(3.39554618e0, 2.71263292e1), 4.65609248e-2, 1e-10);
    }

    #[test]
    fn lift() {
        let table = Table2D::new(
            // The -4 is a 4 in the example. But the tests show, this must be a mistake
            // Otherways, the function does not even work
            [-20., -4., 0., 5., 10., 20., 30.],
            [0., 0.2, 0.6, 0.8, 0.9, 1.3, 1.5, 2., 2.48, 3., 3.9, 40.],
            [
                [
                    -0.07, -0.08, -0.12, -0.12, -0.12, -0.12, -0.12, -0.13, -0.14, -0.12, -0.1,
                    -0.14,
                ],
                [
                    -0.07, -0.08, -0.12, -0.12, -0.12, -0.12, -0.12, -0.13, -0.14, -0.12, -0.1,
                    -0.14,
                ],
                [
                    // The last entry seems to be an input mistake in the example data
                    0.08, 0.08, 0.08, 0.06, 0.06, 0.07, 0.04, 0.0, -0.02, -0.03, -0.04, 0.03,
                ],
                [
                    0.29, 0.29, 0.29, 0.28, 0.28, 0.3, 0.24, 0.17, 0.12, 0.09, 0.08, 0.21,
                ],
                [
                    0.5, 0.6, 0.49, 0.48, 0.52, 0.52, 0.41, 0.33, 0.25, 0.2, 0.15, 0.4,
                ],
                [
                    0.94, 0.94, 0.92, 0.9, 0.94, 0.89, 0.75, 0.68, 0.67, 0.65, 0.62, 0.76,
                ],
                [
                    0.94, 0.94, 0.92, 0.9, 0.94, 0.89, 0.75, 0.68, 0.67, 0.65, 0.62, 0.76,
                ],
            ],
        );

        assert_almost_eq!(table.at(4.72845235e0, 2.47287807e1), 1.46609433e-1, 1e-9);
        assert_almost_eq!(table.at(3.39554618e0, 2.71263292e1), 1.12746181e-1, 1e-9);
    }
}
