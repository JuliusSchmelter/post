// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 14.12.23
// Last modified by Tibor Völcker on 06.03.24
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

#[derive(Debug, Clone)]
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
