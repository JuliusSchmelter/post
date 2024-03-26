// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 14.12.23
// Last modified by Tibor Völcker on 26.03.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use dyn_clone::DynClone;
use serde::Deserialize;
use std::fmt::Debug;

use crate::{state::StateVariable, State};

mod deserialization;
mod linear_interpolation;

pub use init::{Table1D, Table2D, Table3D};

#[derive(Debug, Default, Clone, Copy, Deserialize)]
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
        pub fn new(
            x: (StateVariable, Box<[f64]>),
            data: Box<[f64]>,
            interpolator: Interpolator,
        ) -> Self {
            if !is_sorted(&x.1) {
                panic!("Indices must be sorted by indexing value")
            }
            if x.1.len() != data.len() {
                panic!("Data has invalid length")
            }
            Self {
                x,
                data,
                interpolator,
            }
        }

        pub fn from_static_data<const X: usize>(
            x: (StateVariable, [f64; X]),
            data: [f64; X],
            interpolator: Interpolator,
        ) -> Self {
            Self::new((x.0, x.1.into()), data.into(), interpolator)
        }
    }

    impl Table2D {
        pub fn new(
            x: (StateVariable, Box<[f64]>),
            y: (StateVariable, Box<[f64]>),
            data: &[Box<[f64]>],
            interpolator: Interpolator,
        ) -> Self {
            if !is_sorted(&x.1) {
                panic!("Indices must be sorted by indexing value")
            }
            if x.1.len() != data.len() {
                panic!("Data has invalid length")
            }
            Self {
                x,
                data: data
                    .iter()
                    .map(|row| Table1D::new(y.clone(), row.clone(), interpolator))
                    .collect(),
                interpolator,
            }
        }

        pub fn from_static_data<const X: usize, const Y: usize>(
            x: (StateVariable, [f64; X]),
            y: (StateVariable, [f64; Y]),
            data: [[f64; Y]; X],
            interpolator: Interpolator,
        ) -> Self {
            let data: [Box<[f64]>; X] = data.map(|i| i.into());
            Self::new((x.0, x.1.into()), (y.0, y.1.into()), &data, interpolator)
        }
    }

    impl Table3D {
        pub fn new(
            x: (StateVariable, Box<[f64]>),
            y: (StateVariable, Box<[f64]>),
            z: (StateVariable, Box<[f64]>),
            data: &[Box<[Box<[f64]>]>],
            interpolator: Interpolator,
        ) -> Self {
            if !is_sorted(&x.1) {
                panic!("Indices must be sorted by indexing value")
            }
            if x.1.len() != data.len() {
                panic!("Data has invalid length")
            }
            Self {
                x,
                data: data
                    .iter()
                    .map(|row| Table2D::new(y.clone(), z.clone(), row, interpolator))
                    .collect(),
                interpolator,
            }
        }

        pub fn from_static_data<const X: usize, const Y: usize, const Z: usize>(
            x: (StateVariable, [f64; X]),
            y: (StateVariable, [f64; Y]),
            z: (StateVariable, [f64; Z]),
            data: [[[f64; Z]; Y]; X],
            interpolator: Interpolator,
        ) -> Self {
            let data: [[Box<[f64]>; Y]; X] = data.map(|i| i.map(|i| i.into()));
            let data: [Box<[Box<[f64]>]>; X] = data.map(|i| i.into());
            Self::new(
                (x.0, x.1.into()),
                (y.0, y.1.into()),
                (z.0, z.1.into()),
                &data,
                interpolator,
            )
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        #[should_panic(expected = "Indices must be sorted by indexing value")]
        fn not_sorted() {
            Table1D::from_static_data(
                (StateVariable::Time, [0., 0.]),
                [10., 20.],
                Interpolator::default(),
            );
        }
    }
}
