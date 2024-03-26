// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 14.12.23
// Last modified by Tibor Völcker on 26.03.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use dyn_clone::DynClone;
use std::fmt::Debug;

use crate::{state::StateVariable, State};

pub mod linear_interpolation;

pub use init::{Table1D, Table2D, Table3D};

#[derive(Debug, Default, Clone, Copy)]
pub enum Interpolator {
    #[default]
    Linear,
}

pub trait Table: Debug + DynClone + Sync {
    fn at_state(&self, state: &State) -> f64;
}

impl Default for Box<dyn Table> {
    fn default() -> Self {
        Box::<Table1D>::default()
    }
}

dyn_clone::clone_trait_object!(Table);

#[derive(Debug, Default, Clone)]
pub struct GenericTable<T> {
    x: (StateVariable, Box<[f64]>),
    data: Box<[T]>,
    interpolator: Interpolator,
}

mod init {
    use super::*;

    // We define 3 concrete types to make initialization possible
    pub type Table1D = GenericTable<f64>;
    pub type Table2D = GenericTable<Table1D>;
    pub type Table3D = GenericTable<Table2D>;

    fn is_sorted<T>(data: &[T]) -> bool
    where
        T: PartialOrd,
    {
        data.windows(2).all(|w| w[0] < w[1])
    }

    impl Table1D {
        pub fn new<const X: usize>(x: (StateVariable, [f64; X]), data: [f64; X]) -> Self {
            if !is_sorted(&x.1) {
                panic!("Indices must be sorted by indexing value")
            }
            Self {
                x: (x.0, Box::new(x.1)),
                data: Box::new(data),
                interpolator: Interpolator::Linear,
            }
        }
    }

    impl Table2D {
        pub fn new<const X: usize, const Y: usize>(
            x: (StateVariable, [f64; X]),
            y: (StateVariable, [f64; Y]),
            data: [[f64; Y]; X],
        ) -> Self {
            if !is_sorted(&x.1) {
                panic!("Indices must be sorted by indexing value")
            }
            Self {
                x: (x.0, Box::new(x.1)),
                data: Box::new(data.map(|row| Table1D::new(y, row))),
                interpolator: Interpolator::Linear,
            }
        }
    }

    impl Table3D {
        pub fn new<const X: usize, const Y: usize, const Z: usize>(
            x: (StateVariable, [f64; X]),
            y: (StateVariable, [f64; Y]),
            z: (StateVariable, [f64; Z]),
            data: [[[f64; Z]; Y]; X],
        ) -> Self {
            if !is_sorted(&x.1) {
                panic!("Indices must be sorted by indexing value")
            }
            Self {
                x: (x.0, Box::new(x.1)),
                data: Box::new(data.map(|row| Table2D::new(y, z, row))),
                interpolator: Interpolator::Linear,
            }
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        #[should_panic(expected = "Indices must be sorted by indexing value")]
        fn not_sorted() {
            Table1D::new((StateVariable::Time, [0., 0.]), [10., 20.]);
        }
    }
}
