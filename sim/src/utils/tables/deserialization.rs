// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 25.03.24
// Last modified by Tibor Völcker on 26.03.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)

use super::*;
use serde::{de, Deserialize};

fn get_var<'de, A>(map: &mut A) -> Result<(StateVariable, Box<[f64]>), A::Error>
where
    A: de::MapAccess<'de>,
{
    if let Some((key, value)) = map.next_entry()? {
        Ok((key, value))
    } else {
        Err(de::Error::missing_field("variable name"))
    }
}

fn get_data<'de, A, T: Deserialize<'de>>(map: &mut A) -> Result<T, A::Error>
where
    A: de::MapAccess<'de>,
{
    if let Some((key, value)) = map.next_entry()? {
        if key != "data" {
            return Err(de::Error::unknown_field(key, &["data"]));
        }
        Ok(value)
    } else {
        Err(de::Error::missing_field("data"))
    }
}

fn get_interpolator<'de, A>(map: &mut A) -> Result<Interpolator, A::Error>
where
    A: de::MapAccess<'de>,
{
    if let Some((key, value)) = map.next_entry()? {
        if key != "interpolator" {
            return Err(de::Error::unknown_field(key, &["interpolator"]));
        }
        Ok(value)
    } else {
        Ok(Interpolator::default())
    }
}

impl<'de> de::Deserialize<'de> for Table1D {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: de::Deserializer<'de>,
    {
        struct Table1DVisitor;

        impl<'de> de::Visitor<'de> for Table1DVisitor {
            type Value = Table1D;

            fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
                formatter.write_str("1D Table")
            }

            fn visit_map<A>(self, mut map: A) -> Result<Self::Value, A::Error>
            where
                A: de::MapAccess<'de>,
            {
                let x = get_var(&mut map)?;

                let data = get_data(&mut map)?;

                let interpolator = get_interpolator(&mut map)?;

                Ok(Self::Value::new(x, data, interpolator))
            }
        }

        deserializer.deserialize_map(Table1DVisitor {})
    }
}

impl<'de> de::Deserialize<'de> for Table2D {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: de::Deserializer<'de>,
    {
        struct Table2DVisitor;

        impl<'de> de::Visitor<'de> for Table2DVisitor {
            type Value = Table2D;

            fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
                formatter.write_str("2D Table")
            }

            fn visit_map<A>(self, mut map: A) -> Result<Self::Value, A::Error>
            where
                A: de::MapAccess<'de>,
            {
                let x = get_var(&mut map)?;

                let y = get_var(&mut map)?;

                let data: Box<[Box<[f64]>]> = get_data(&mut map)?;

                let interpolator = get_interpolator(&mut map)?;

                Ok(Self::Value::new(x, y, &data, interpolator))
            }
        }

        deserializer.deserialize_map(Table2DVisitor {})
    }
}

impl<'de> de::Deserialize<'de> for Table3D {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: de::Deserializer<'de>,
    {
        struct Table3DVisitor;

        impl<'de> de::Visitor<'de> for Table3DVisitor {
            type Value = Table3D;

            fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
                formatter.write_str("3D Table")
            }

            fn visit_map<A>(self, mut map: A) -> Result<Self::Value, A::Error>
            where
                A: de::MapAccess<'de>,
            {
                let x = get_var(&mut map)?;

                let y = get_var(&mut map)?;

                let z = get_var(&mut map)?;

                #[allow(clippy::type_complexity)]
                let data: Box<[Box<[Box<[f64]>]>]> = get_data(&mut map)?;

                let interpolator = get_interpolator(&mut map)?;

                Ok(Self::Value::new(x, y, z, &data, interpolator))
            }
        }

        deserializer.deserialize_map(Table3DVisitor {})
    }
}
