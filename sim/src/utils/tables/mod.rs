// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 14.12.23
// Last modified by Tibor Völcker on 22.05.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use std::fmt::Debug;

use serde::Deserialize;

use crate::state::{State, StateVariable};

mod deserialization;
mod linear_interpolation;

// Enum needed for generic deserialization
// Sadly, untagged enums swallow precise error messages
#[derive(Debug, Clone, Deserialize, PartialEq)]
#[serde(untagged)]
pub enum Table {
    D1(Table1D),
    D2(Table2D),
    D3(Table3D),
}

impl Default for Table {
    fn default() -> Self {
        Self::D1(Table1D::default())
    }
}

impl Table {
    pub fn at_state(&self, state: &State) -> f64 {
        match self {
            Self::D1(table) => table.at_state(state),
            Self::D2(table) => table.at_state(state),
            Self::D3(table) => table.at_state(state),
        }
    }
}

#[derive(Debug, Default, Clone, Deserialize, PartialEq)]
#[serde(try_from = "deserialization::Table1DUnchecked")]
pub struct Table1D {
    x: (StateVariable, Box<[f64]>),
    data: Box<[f64]>,
    interpolator: Interpolator,
}

#[derive(Debug, Default, Clone, Deserialize, PartialEq)]
#[serde(try_from = "deserialization::Table2DUnchecked")]
pub struct Table2D {
    x: (StateVariable, Box<[f64]>),
    y: (StateVariable, Box<[f64]>),
    data: Box<[Box<[f64]>]>,
    interpolator: Interpolator,
}

#[derive(Debug, Default, Clone, Deserialize, PartialEq)]
#[serde(try_from = "deserialization::Table3DUnchecked")]
pub struct Table3D {
    x: (StateVariable, Box<[f64]>),
    y: (StateVariable, Box<[f64]>),
    z: (StateVariable, Box<[f64]>),
    #[allow(clippy::type_complexity)]
    data: Box<[Box<[Box<[f64]>]>]>,
    interpolator: Interpolator,
}

#[derive(Debug, Default, Clone, Copy, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum Interpolator {
    #[default]
    Linear,
}

mod interpolate {
    use super::*;

    impl Table1D {
        pub fn at_state(&self, state: &State) -> f64 {
            match self.interpolator {
                Interpolator::Linear => {
                    let x = self.x.0.get_value(state);
                    linear_interpolation::linear_interpolate(&self.x.1, x, &self.data)
                }
            }
        }
    }

    impl Table2D {
        pub fn at_state(&self, state: &State) -> f64 {
            match self.interpolator {
                Interpolator::Linear => {
                    let x = self.x.0.get_value(state);
                    let y = self.y.0.get_value(state);
                    linear_interpolation::bilinear_interpolate(
                        &self.x.1, x, &self.y.1, y, &self.data,
                    )
                }
            }
        }
    }

    impl Table3D {
        pub fn at_state(&self, state: &State) -> f64 {
            match self.interpolator {
                Interpolator::Linear => {
                    let x = self.x.0.get_value(state);
                    let y = self.y.0.get_value(state);
                    let z = self.z.0.get_value(state);
                    linear_interpolation::trilinear_interpolate(
                        &self.x.1, x, &self.y.1, y, &self.z.1, z, &self.data,
                    )
                }
            }
        }
    }
}

mod init {
    use super::*;
    use std::fmt::Display;

    #[derive(Debug, PartialEq)]
    pub enum TableInitError {
        NotSortedError,
        InvalidLengthError,
    }

    impl Display for TableInitError {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            match self {
                TableInitError::NotSortedError => {
                    write!(f, "Table argument values are not sorted")
                }
                TableInitError::InvalidLengthError => {
                    write!(f, "Argument length and data length does not match")
                }
            }
        }
    }

    fn validate<T>(idx_arr: &[f64], data_arr: &[T]) -> Result<(), TableInitError> {
        if !idx_arr.windows(2).all(|w| w[0] < w[1]) {
            Err(TableInitError::NotSortedError)
        } else if idx_arr.len() != data_arr.len() {
            Err(TableInitError::InvalidLengthError)
        } else {
            Ok(())
        }
    }

    impl Table1D {
        pub fn try_new(
            x: (StateVariable, &[f64]),
            data: &[f64],
            interpolator: Interpolator,
        ) -> Result<Self, TableInitError> {
            validate(x.1, data)?;

            Ok(Self {
                x: (x.0, x.1.into()),
                data: data.into(),
                interpolator,
            })
        }
    }

    impl Table2D {
        pub fn try_new(
            x: (StateVariable, &[f64]),
            y: (StateVariable, &[f64]),
            data: &[Box<[f64]>],
            interpolator: Interpolator,
        ) -> Result<Self, TableInitError> {
            validate(x.1, data)?;

            for y_data in data.iter() {
                validate(y.1, y_data)?;
            }

            Ok(Self {
                x: (x.0, x.1.into()),
                y: (y.0, y.1.into()),
                data: data.into(),
                interpolator,
            })
        }
    }

    impl Table3D {
        pub fn try_new(
            x: (StateVariable, &[f64]),
            y: (StateVariable, &[f64]),
            z: (StateVariable, &[f64]),
            data: &[Box<[Box<[f64]>]>],
            interpolator: Interpolator,
        ) -> Result<Self, TableInitError> {
            validate(x.1, data)?;

            for y_data in data.iter() {
                validate(y.1, y_data)?;

                for z_data in y_data.iter() {
                    validate(z.1, z_data)?;
                }
            }

            Ok(Self {
                x: (x.0, x.1.into()),
                y: (y.0, y.1.into()),
                z: (z.0, z.1.into()),
                data: data.into(),
                interpolator,
            })
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn not_sorted() {
            let result = Table1D::try_new(
                (StateVariable::Time, &[0., 0.]),
                &[10., 20.],
                Interpolator::default(),
            )
            .unwrap_err();

            assert_eq!(result, TableInitError::NotSortedError);
        }

        #[test]
        fn invalid_length() {
            let result = Table1D::try_new(
                (StateVariable::Time, &[0., 1.]),
                &[10., 20., 30.],
                Interpolator::default(),
            )
            .unwrap_err();

            assert_eq!(result, TableInitError::InvalidLengthError);
        }

        #[test]
        fn not_sorted_2d_x() {
            let result = Table2D::try_new(
                (StateVariable::Time, &[0., -1.]),
                (StateVariable::Time, &[0., 1.]),
                &[[10., 20.].into(), [10., 20.].into()],
                Interpolator::default(),
            )
            .unwrap_err();

            assert_eq!(result, TableInitError::NotSortedError);
        }

        #[test]
        fn not_sorted_2d_y() {
            let result = Table2D::try_new(
                (StateVariable::Time, &[0., 1.]),
                (StateVariable::Time, &[0., 0.]),
                &[[10., 20.].into(), [10., 20.].into()],
                Interpolator::default(),
            )
            .unwrap_err();

            assert_eq!(result, TableInitError::NotSortedError);
        }

        #[test]
        fn invalid_length_2d_x() {
            let result = Table2D::try_new(
                (StateVariable::Time, &[0., 1.]),
                (StateVariable::Time, &[0., 1.]),
                &[[10., 20.].into(), [10., 20.].into(), [10., 20.].into()],
                Interpolator::default(),
            )
            .unwrap_err();

            assert_eq!(result, TableInitError::InvalidLengthError);
        }

        #[test]
        fn invalid_length_2d_y() {
            let result = Table2D::try_new(
                (StateVariable::Time, &[0., 1.]),
                (StateVariable::Time, &[0., 1.]),
                &[[10., 20.].into(), [10., 20., 30.].into()],
                Interpolator::default(),
            )
            .unwrap_err();

            assert_eq!(result, TableInitError::InvalidLengthError);
        }

        #[test]
        fn invalid_length_3d_z() {
            let result = Table3D::try_new(
                (StateVariable::Time, &[0.]),
                (StateVariable::Time, &[0., 1.]),
                (StateVariable::Time, &[0., 1.]),
                &[[[10., 20., 30.].into(), [10., 20.].into()].into()],
                Interpolator::default(),
            )
            .unwrap_err();

            assert_eq!(result, TableInitError::InvalidLengthError);
        }
    }
}
