// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 14.12.23
// Last modified by Tibor Völcker on 24.05.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

//! Defines the [`Table`] enum, which defines the three types of Tables: 1D, 2D
//! and 3D.

mod deserialization;
mod linear_interpolation;

use crate::state::{State, StateVariable};
use serde::Deserialize;
use std::fmt::Debug;

/// Represents a table.
///
/// The interpolated needs to be an enum (instead of a trait), so that it can
/// be deserialized into any table, depending on user input.
/// Sadly, untagged enums swallow precise error messages.
#[derive(Debug, Clone, Deserialize, PartialEq)]
#[serde(untagged)]
pub enum Table {
    /// A 1D table which is interpolated with one state variable.
    D1(Table1D),
    /// A 2D table which is interpolated with two state variables.
    D2(Table2D),
    /// A 3D table which is interpolated with three state variables.
    D3(Table3D),
}

impl Default for Table {
    /// The default table is an empty 1D table.
    fn default() -> Self {
        Self::D1(Table1D::default())
    }
}

impl Table {
    /// Retrieves a value from the table by interpolating based on the
    /// specified variables in the given state.
    pub fn at_state(&self, state: &State) -> f64 {
        match self {
            Self::D1(table) => table.at_state(state),
            Self::D2(table) => table.at_state(state),
            Self::D3(table) => table.at_state(state),
        }
    }
}

/// Represents a 1D table.
///
/// It is interpolated with one state variable, specified in the `"x"` field.
///
/// The deserialization is handled with [`deserialization::Table1DUnchecked`].
#[derive(Debug, Default, Clone, Deserialize, PartialEq)]
#[serde(try_from = "deserialization::Table1DUnchecked")]
pub struct Table1D {
    /// The state variable and its bases to interpolate with.
    x: (StateVariable, Box<[f64]>),
    /// The table data. One entry corresponds to each state variable base.
    data: Box<[f64]>,
    /// The type of interpolation.
    interpolator: Interpolator,
}

/// Represents a 2D table.
///
/// It is interpolated with two state variable, specified in the `"x"` and
/// `"y"` fields.
///
/// The deserialization is handled with [`deserialization::Table2DUnchecked`].
#[derive(Debug, Default, Clone, Deserialize, PartialEq)]
#[serde(try_from = "deserialization::Table2DUnchecked")]
pub struct Table2D {
    /// The first state variable and its bases to interpolate with.
    x: (StateVariable, Box<[f64]>),
    /// The second state variable and its bases to interpolate with.
    y: (StateVariable, Box<[f64]>),
    /// The table data. The first array must have the same length as the first
    /// state variable bases, each inner array must have the same length as the
    /// second state variable bases.
    data: Box<[Box<[f64]>]>,
    /// The type of interpolation.
    interpolator: Interpolator,
}

/// Represents a 3D table.
///
/// It is interpolated with two state variable, specified in the `"x"`, `"y"`
/// and `"z"` fields.
///
/// The deserialization is handled with [`deserialization::Table3DUnchecked`].
#[derive(Debug, Default, Clone, Deserialize, PartialEq)]
#[serde(try_from = "deserialization::Table3DUnchecked")]
pub struct Table3D {
    /// The first state variable and its bases to interpolate with.
    x: (StateVariable, Box<[f64]>),
    /// The second state variable and its bases to interpolate with.
    y: (StateVariable, Box<[f64]>),
    /// The third state variable and its bases to interpolate with.
    z: (StateVariable, Box<[f64]>),
    /// The table data. The first array must have the same length as the first
    /// state variable bases, each inner array must have the same length as the
    /// second state variable bases and the innermost array must have the same
    /// length as the third state variable bases.
    #[allow(clippy::type_complexity)]
    data: Box<[Box<[Box<[f64]>]>]>,
    /// The type of interpolation.
    interpolator: Interpolator,
}

/// Defines the interpolation type.
///
/// For now only includes linear interpolation, but cubic interpolation
/// can be added in the future.
#[derive(Debug, Default, Clone, Copy, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum Interpolator {
    #[default]
    /// Use linear interpolation between the bases.
    Linear,
}

mod interpolate {
    //! Handles the interpolation. Defines the `at_state` function for each
    //! table type, which matches the interpolator and calls the corresponding
    //! interpolation function.

    use super::*;

    impl Table1D {
        /// Interpolate the table data with the specified state variable in
        /// `state`.
        ///
        /// Linear interpolation uses [`linear_interpolation::linear_interpolate`].
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
        /// Interpolate the table data with the specified state variables in
        /// `state`.
        ///
        /// Linear interpolation uses [`linear_interpolation::bilinear_interpolate`].
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
        /// Interpolate the table data with the specified state variables in
        /// `state`.
        ///
        /// Linear interpolation uses [`linear_interpolation::trilinear_interpolate`].
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
    //! Handles the initialization of the tables. Defines the `try_new`
    //! function for each table type, which tries to create a valid table from
    //! unchecked data. This is used for deserialization from the unchecked
    //! table types.

    use super::*;
    use std::fmt::Display;

    /// Represents an error in initialization.
    #[derive(Debug, PartialEq)]
    pub enum TableInitError {
        /// The state variables were not sorted.
        ///
        /// They must be sorted for easier interpolation.
        NotSortedError,
        /// The data and state variable lengths did not match.
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

    /// Helper function to validate a state variable and its corresponding
    /// table data.
    ///
    /// Checks whether the two arrays have the same length and if the state
    /// variable array is sorted.
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
        /// Tries to create a 1D table from unchecked data. Checks wether the
        /// array in `"x"` is sorted, and its length matches the one in the
        /// `"data"` field.
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
        /// Tries to create a 2D table from unchecked data. Checks wether the
        /// arrays in `"x"` and `"y"` are sorted, and their lengths match the
        /// corresponding one in the `"data"` field.
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
        /// Tries to create a 3D table from unchecked data. Checks wether the
        /// arrays in `"x"`, `"z"` and `"y"` are sorted, and their lengths
        /// match the corresponding one in the `"data"` field.
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
