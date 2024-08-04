// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 25.03.24
// Last modified by Tibor Völcker on 24.05.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)

//! Handles the deserialization of the tables. It uses serde's `derive`.

use super::init::TableInitError;
use super::*;
use serde::Deserialize;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TableUnchecked {
    vars: Vec<(StateVariable, Box<[f64]>)>,
    data: TableData,
    #[serde(default)]
    interpolator: Interpolator,
}

impl TryFrom<TableUnchecked> for Table {
    type Error = TableInitError;

    fn try_from(table: TableUnchecked) -> Result<Self, Self::Error> {
        Self::try_new(table.vars, table.data, table.interpolator)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_table() {
        let input = r#"{"vars": [], "data": []}"#;
        serde_json::from_str::<Table>(input).unwrap_err();
    }

    #[test]
    fn empty_table_with_interpolator() {
        let input = r#"{"vars": [["time", []]], "data": [], "interpolator": "linear"}"#;
        let table: Table = serde_json::from_str(input).unwrap();

        assert_eq!(
            table,
            Table::try_new(
                vec![(StateVariable::Time, [].into())],
                TableData::Values([].into()),
                Interpolator::Linear
            )
            .unwrap()
        )
    }

    #[test]
    fn invalid_iterator() {
        let input = r#"{"vars": [["time", [0.0]]], "data": [0.0], "interpolator": "not_there"}"#;
        serde_json::from_str::<Table>(input).unwrap_err();
    }

    #[test]
    fn invalid_length() {
        let input = r#"{"vars": [["time", [0.0, 1.0]]], "data": [0.0]}"#;
        serde_json::from_str::<Table>(input).unwrap_err();
    }

    #[test]
    fn example_1d() {
        let input = r#"{"vars": [["time", [0.0]]], "data": [1.0]}"#;
        let table: Table = serde_json::from_str(input).unwrap();

        assert_eq!(
            table,
            Table::try_new(
                vec![(StateVariable::Time, [0.].into())],
                TableData::Values([1.].into()),
                Interpolator::Linear
            )
            .unwrap()
        );
    }

    #[test]
    fn example_2d() {
        let input = r#"{"vars": [["time", [0.0]], ["mass", [0.0]]], "data": [[1.0]]}"#;
        let table: Table = serde_json::from_str(input).unwrap();

        assert_eq!(
            table,
            Table::try_new(
                vec![
                    (StateVariable::Time, [0.].into()),
                    (StateVariable::Mass, [0.].into())
                ],
                TableData::Table([TableData::Values([1.].into())].into()),
                Interpolator::Linear
            )
            .unwrap()
        );
    }

    #[test]
    fn example_3d() {
        let input = r#"{"vars": [["time", [0.0]], ["mass", [0.0, 1.0]], ["altitude", [0.0]]], "data": [[[1.0], [2.0]]]}"#;
        let table: Table = serde_json::from_str(input).unwrap();

        assert_eq!(
            table,
            Table::try_new(
                vec![
                    (StateVariable::Time, [0.].into()),
                    (StateVariable::Mass, [0., 1.].into()),
                    (StateVariable::Altitude, [0.].into()),
                ],
                TableData::Table(
                    [TableData::Table(
                        [
                            TableData::Values([1.].into()),
                            TableData::Values([2.].into())
                        ]
                        .into()
                    )]
                    .into()
                ),
                Interpolator::Linear
            )
            .unwrap()
        );
    }
}
