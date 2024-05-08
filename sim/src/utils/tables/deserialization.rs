// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 25.03.24
// Last modified by Tibor Völcker on 08.05.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)

use super::init::TableInitError;
use super::*;
use serde::Deserialize;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Table1DUnchecked {
    x: (StateVariable, Box<[f64]>),
    data: Box<[f64]>,
    #[serde(default)]
    interpolator: Interpolator,
}

impl TryFrom<Table1DUnchecked> for Table1D {
    type Error = TableInitError;

    fn try_from(value: Table1DUnchecked) -> Result<Self, Self::Error> {
        Self::try_new((value.x.0, &value.x.1), &value.data, value.interpolator)
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Table2DUnchecked {
    x: (StateVariable, Box<[f64]>),
    y: (StateVariable, Box<[f64]>),
    data: Box<[Box<[f64]>]>,
    #[serde(default)]
    interpolator: Interpolator,
}

impl TryFrom<Table2DUnchecked> for Table2D {
    type Error = TableInitError;

    fn try_from(value: Table2DUnchecked) -> Result<Self, Self::Error> {
        Self::try_new(
            (value.x.0, &value.x.1),
            (value.y.0, &value.y.1),
            &value.data,
            value.interpolator,
        )
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Table3DUnchecked {
    x: (StateVariable, Box<[f64]>),
    y: (StateVariable, Box<[f64]>),
    z: (StateVariable, Box<[f64]>),
    #[allow(clippy::type_complexity)]
    data: Box<[Box<[Box<[f64]>]>]>,
    #[serde(default)]
    interpolator: Interpolator,
}

impl TryFrom<Table3DUnchecked> for Table3D {
    type Error = TableInitError;

    fn try_from(value: Table3DUnchecked) -> Result<Self, Self::Error> {
        Self::try_new(
            (value.x.0, &value.x.1),
            (value.y.0, &value.y.1),
            (value.z.0, &value.z.1),
            &value.data,
            value.interpolator,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_table() {
        let input = r#"{"x": ["time", []], "data": []}"#;
        let table: Table = serde_json::from_str(input).unwrap();

        assert_eq!(table, Table::D1(Table1D::default()))
    }

    #[test]
    fn empty_table_with_interpolator() {
        let input = r#"{"x": ["time", []], "data": [], "interpolator": "linear"}"#;
        let table: Table = serde_json::from_str(input).unwrap();

        assert_eq!(table, Table::D1(Table1D::default()))
    }

    #[test]
    fn invalid_iterator() {
        let input = r#"{"x": ["time", [0.0]], "data": [0.0], "interpolator": "not_there"}"#;
        serde_json::from_str::<Table>(input).unwrap_err();
    }

    #[test]
    fn invalid_length() {
        let input = r#"{"x": ["time", [0.0, 1.0]], "data": [0.0]}"#;
        serde_json::from_str::<Table>(input).unwrap_err();
    }

    #[test]
    fn example_1d() {
        let input = r#"{"x": ["time", [0.0]], "data": [1.0]}"#;
        let table: Table = serde_json::from_str(input).unwrap();

        assert_eq!(
            table,
            Table::D1(
                Table1D::try_new((StateVariable::Time, &[0.]), &[1.], Interpolator::Linear)
                    .unwrap()
            )
        );
    }

    #[test]
    fn example_2d() {
        let input = r#"{"x": ["time", [0.0]], "y": ["mass", [0.0]], "data": [[1.0]]}"#;
        let table: Table = serde_json::from_str(input).unwrap();

        assert_eq!(
            table,
            Table::D2(
                Table2D::try_new(
                    (StateVariable::Time, &[0.]),
                    (StateVariable::Mass, &[0.]),
                    &[[1.].into()],
                    Interpolator::Linear
                )
                .unwrap()
            )
        );
    }

    #[test]
    fn example_3d() {
        let input = r#"{"x": ["time", [0.0]], "y": ["mass", [0.0, 1.0]], "z": ["altitude", [0.0]], "data": [[[1.0], [2.0]]]}"#;
        let table: Table = serde_json::from_str(input).unwrap();

        assert_eq!(
            table,
            Table::D3(
                Table3D::try_new(
                    (StateVariable::Time, &[0.]),
                    (StateVariable::Mass, &[0., 1.]),
                    (StateVariable::Altitude, &[0.]),
                    &[[[1.].into(), [2.].into()].into()],
                    Interpolator::Linear
                )
                .unwrap()
            )
        );
    }
}
