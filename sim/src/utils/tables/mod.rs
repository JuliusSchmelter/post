// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 14.12.23
// Last modified by Tibor Völcker on 24.05.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

//! Defines the [`Table`] struct.

mod deserialization;
mod linear_interpolation;

use crate::state::{State, StateVariable};
use serde::Deserialize;
use std::{default, fmt::Debug};

/// Represents a table of arbitrary dimension.
#[derive(Debug, Clone, Deserialize, PartialEq, Default)]
#[serde(try_from = "deserialization::TableUnchecked")]
pub struct Table {
    /// The first state variables and its bases to interpolate with.
    vars: Vec<(StateVariable, Box<[f64]>)>,
    /// The table data. The first array must have the same length as the first
    /// state variable bases, each inner array must have the same length as the
    /// second state variable bases and the innermost array must have the same
    /// length as the third state variable bases.
    data: TableData,
    /// The type of interpolation.
    interpolator: Interpolator,
}

/// Holds the data of a table. Arbirarily nested arrays of f64s.
#[derive(Debug, Clone, Deserialize, PartialEq)]
#[serde(untagged)]
pub enum TableData {
    Values(Box<[f64]>),
    Table(Box<[Self]>),
}

impl default::Default for TableData {
    fn default() -> Self {
        Self::Values(Box::default())
    }
}

impl Table {
    /// Retrieves a value from the table by interpolating based on the
    /// specified variables in the given state.
    pub fn at_state(&self, state: &State) -> f64 {
        match self.interpolator {
            Interpolator::Linear => {
                let (at, bases): (Vec<_>, Vec<_>) = self
                    .vars
                    .iter()
                    .map(|(var, bases)| (var.get_value(state), bases.as_ref()))
                    .unzip();

                linear_interpolation::interpolate(&bases, &at, &self.data)
            }
        }
    }
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
    fn validate(
        vars: &[(StateVariable, Box<[f64]>)],
        data: &TableData,
    ) -> Result<(), TableInitError> {
        // Tables with zero vars are invalid.
        if vars.len() == 0 {
            Err(TableInitError::InvalidLengthError)?
        }

        if !vars[0].1.windows(2).all(|w| w[0] < w[1]) {
            Err(TableInitError::NotSortedError)?
        }

        match data {
            TableData::Values(values) => {
                if vars.len() != 1 {
                    Err(TableInitError::InvalidLengthError)?
                } else if vars[0].1.len() != values.len() {
                    Err(TableInitError::InvalidLengthError)?
                }
            }
            TableData::Table(table) => {
                if vars[0].1.len() != table.len() {
                    Err(TableInitError::InvalidLengthError)?
                }

                for elem in table.iter() {
                    validate(&vars[1..], elem)?;
                }
            }
        }

        Ok(())
    }

    impl Table {
        /// Tries to create a table from unchecked data. Checks whether the
        /// arrays in the state variables are sorted, and their lengths match
        /// the corresponding one in the `"data"` field.
        pub fn try_new(
            vars: Vec<(StateVariable, Box<[f64]>)>,
            data: TableData,
            interpolator: Interpolator,
        ) -> Result<Self, TableInitError> {
            validate(&vars, &data)?;

            Ok(Self {
                vars,
                data,
                interpolator,
            })
        }
    }

    #[cfg(test)]
    mod tests {
        use std::vec;

        use super::*;

        #[test]
        fn not_sorted() {
            let result = Table::try_new(
                vec![(StateVariable::Time, [0., 0.].into())],
                TableData::Values([10., 20.].into()),
                Interpolator::default(),
            )
            .unwrap_err();

            assert_eq!(result, TableInitError::NotSortedError);
        }

        #[test]
        fn invalid_length() {
            let result = Table::try_new(
                vec![(StateVariable::Time, [0., 1.].into())],
                TableData::Values([10., 20., 30.].into()),
                Interpolator::default(),
            )
            .unwrap_err();

            assert_eq!(result, TableInitError::InvalidLengthError);
        }

        #[test]
        fn not_sorted_2d_x() {
            let result = Table::try_new(
                vec![
                    (StateVariable::Time, [0., -1.].into()),
                    (StateVariable::Time, [0., 1.].into()),
                ],
                TableData::Table(
                    [
                        TableData::Values([10., 20.].into()),
                        TableData::Values([10., 20.].into()),
                    ]
                    .into(),
                ),
                Interpolator::default(),
            )
            .unwrap_err();

            assert_eq!(result, TableInitError::NotSortedError);
        }

        #[test]
        fn not_sorted_2d_y() {
            let result = Table::try_new(
                vec![
                    (StateVariable::Time, [0., 1.].into()),
                    (StateVariable::Time, [0., 0.].into()),
                ],
                TableData::Table(
                    [
                        TableData::Values([10., 20.].into()),
                        TableData::Values([10., 20.].into()),
                    ]
                    .into(),
                ),
                Interpolator::default(),
            )
            .unwrap_err();

            assert_eq!(result, TableInitError::NotSortedError);
        }

        #[test]
        fn invalid_length_2d_x() {
            let result = Table::try_new(
                vec![
                    (StateVariable::Time, [0., 1.].into()),
                    (StateVariable::Time, [0., 1.].into()),
                ],
                TableData::Table(
                    [
                        TableData::Values([10., 20.].into()),
                        TableData::Values([10., 20.].into()),
                        TableData::Values([10., 20.].into()),
                    ]
                    .into(),
                ),
                Interpolator::default(),
            )
            .unwrap_err();

            assert_eq!(result, TableInitError::InvalidLengthError);
        }

        #[test]
        fn invalid_length_2d_y() {
            let result = Table::try_new(
                vec![
                    (StateVariable::Time, [0., 1.].into()),
                    (StateVariable::Time, [0., 1.].into()),
                ],
                TableData::Table(
                    [
                        TableData::Values([10., 20.].into()),
                        TableData::Values([10., 20., 30.].into()),
                    ]
                    .into(),
                ),
                Interpolator::default(),
            )
            .unwrap_err();

            assert_eq!(result, TableInitError::InvalidLengthError);
        }

        #[test]
        fn invalid_length_3d_z() {
            let result = Table::try_new(
                vec![
                    (StateVariable::Time, [0.].into()),
                    (StateVariable::Time, [0., 1.].into()),
                    (StateVariable::Time, [0., 1.].into()),
                ],
                TableData::Table(
                    [TableData::Table(
                        [
                            TableData::Values([10., 20., 30.].into()),
                            TableData::Values([10., 20.].into()),
                        ]
                        .into(),
                    )]
                    .into(),
                ),
                Interpolator::default(),
            )
            .unwrap_err();

            assert_eq!(result, TableInitError::InvalidLengthError);
        }
    }
}
