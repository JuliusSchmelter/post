// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 14.12.23
// Last modified by Tibor Völcker on 25.03.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use crate::{state::StateVariable, State};

pub mod linear_interpolation;

fn is_sorted<T>(data: &[T]) -> bool
where
    T: PartialOrd,
{
    data.windows(2).all(|w| w[0] < w[1])
}

#[derive(Debug, Default, Clone, Copy)]
pub enum Interpolator {
    #[default]
    Linear,
}

pub trait TableTrait {
    fn at_state(&self, state: &State) -> f64;
}

#[derive(Debug, Default, Clone)]
pub struct Table<T> {
    data: Box<[T]>,
    x: Box<[f64]>,
    interpolator: Interpolator,
    variable: StateVariable,
}

pub type Table1D = Table<f64>;
pub type Table2D = Table<Table1D>;
pub type Table3D = Table<Table2D>;

impl Table1D {
    pub fn new<const X: usize>(x: [f64; X], data: [f64; X], variable: StateVariable) -> Self {
        if !is_sorted(&x) {
            panic!("Data must be sorted by indexing value")
        }
        Self {
            x: Box::new(x),
            data: Box::new(data),
            interpolator: Interpolator::Linear,
            variable,
        }
    }
}

impl Table2D {
    pub fn new<const X: usize, const Y: usize>(
        x: [f64; X],
        y: [f64; Y],
        data: [[f64; Y]; X],
        variables: [StateVariable; 2],
    ) -> Self {
        if !is_sorted(&x) {
            panic!("Data must be sorted by indexing value")
        }
        Self {
            x: Box::new(x),
            data: Box::new(data.map(|row| Table1D::new(y, row, variables[1]))),
            interpolator: Interpolator::Linear,
            variable: variables[0],
        }
    }
}

impl Table3D {
    pub fn new<const X: usize, const Y: usize, const Z: usize>(
        x: [f64; X],
        y: [f64; Y],
        z: [f64; Z],
        data: [[[f64; Z]; Y]; X],
        variables: [StateVariable; 3],
    ) -> Self {
        if !is_sorted(&x) {
            panic!("Data must be sorted by indexing value")
        }
        Self {
            x: Box::new(x),
            data: Box::new(
                data.map(|row| Table2D::new(y, z, row, variables[1..].try_into().unwrap())),
            ),
            interpolator: Interpolator::Linear,
            variable: variables[0],
        }
    }
}
