// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 14.12.23
// Last modified by Tibor Völcker on 06.01.24
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

pub struct Table<T, const X: usize, I: Interpolator> {
    data: [T; X],
    x: [f64; X],
    interpolator: PhantomData<I>,
}

type Table1D<const X: usize, I> = Table<f64, X, I>;
type Table2D<const X: usize, const Y: usize, I> = Table<Table1D<Y, I>, X, I>;
type Table3D<const X: usize, const Y: usize, const Z: usize, I> = Table<Table2D<Y, Z, I>, X, I>;

impl<const X: usize, I: Interpolator> Table1D<X, I> {
    pub fn new(x: [f64; X], data: [f64; X]) -> Self {
        if !is_sorted(&x) {
            panic!("Data must be sorted by indexing value")
        }
        Self {
            x,
            data,
            interpolator: PhantomData,
        }
    }
}

impl<const X: usize, const Y: usize, I: Interpolator> Table2D<X, Y, I> {
    pub fn new(x: [f64; X], y: [f64; Y], data: [[f64; Y]; X]) -> Self {
        if !is_sorted(&y) {
            panic!("Data must be sorted by indexing value")
        }
        Self {
            x,
            data: data.map(|row| Table1D::new(y, row)),
            interpolator: PhantomData,
        }
    }
}

impl<const X: usize, const Y: usize, const Z: usize, I: Interpolator> Table3D<X, Y, Z, I> {
    pub fn new(x: [f64; X], y: [f64; Y], z: [f64; Z], data: [[[f64; Z]; Y]; X]) -> Self {
        if !is_sorted(&y) {
            panic!("Data must be sorted by indexing value")
        }
        Self {
            x,
            data: data.map(|row| Table2D::new(y, z, row)),
            interpolator: PhantomData,
        }
    }
}
