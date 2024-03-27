// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 14.12.23
// Last modified by Tibor Völcker on 27.03.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use std::fmt::Debug;

use crate::{state::StateVariable, State};

mod linear_interpolation;

// Enum needed for generic deserialization
#[derive(Debug, Clone)]
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

#[derive(Debug, Default, Clone)]
pub struct Table1D {
    x: (StateVariable, Box<[f64]>),
    data: Box<[f64]>,
    interpolator: Interpolator,
}

#[derive(Debug, Default, Clone)]
pub struct Table2D {
    x: (StateVariable, Box<[f64]>),
    y: (StateVariable, Box<[f64]>),
    data: Box<[Box<[f64]>]>,
    interpolator: Interpolator,
}

#[derive(Debug, Default, Clone)]
pub struct Table3D {
    x: (StateVariable, Box<[f64]>),
    y: (StateVariable, Box<[f64]>),
    z: (StateVariable, Box<[f64]>),
    #[allow(clippy::type_complexity)]
    data: Box<[Box<[Box<[f64]>]>]>,
    interpolator: Interpolator,
}

#[derive(Debug, Default, Clone, Copy)]
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

    impl Table1D {
        pub fn new<const X: usize>(
            x: (StateVariable, [f64; X]),
            data: [f64; X],
            interpolator: Interpolator,
        ) -> Self {
            Self {
                x: (x.0, x.1.into()),
                data: data.into(),
                interpolator,
            }
            .validate()
        }
    }

    impl Table2D {
        pub fn new<const X: usize, const Y: usize>(
            x: (StateVariable, [f64; X]),
            y: (StateVariable, [f64; Y]),
            data: [[f64; Y]; X],
            interpolator: Interpolator,
        ) -> Self {
            Self {
                x: (x.0, x.1.into()),
                y: (y.0, y.1.into()),
                data: data.map(|i| i.into()).into(),
                interpolator,
            }
            .validate()
        }
    }

    impl Table3D {
        pub fn new<const X: usize, const Y: usize, const Z: usize>(
            x: (StateVariable, [f64; X]),
            y: (StateVariable, [f64; Y]),
            z: (StateVariable, [f64; Z]),
            data: [[[f64; Z]; Y]; X],
            interpolator: Interpolator,
        ) -> Self {
            Self {
                x: (x.0, x.1.into()),
                y: (y.0, y.1.into()),
                z: (z.0, z.1.into()),
                data: data.map(|i| i.map(|i| i.into()).into()).into(),
                interpolator,
            }
            .validate()
        }
    }
}

mod validate {
    use super::*;

    fn validate<T>(idx_arr: &[f64], data_arr: &[T]) {
        assert!(
            idx_arr.windows(2).all(|w| w[0] < w[1]),
            "Indices must be sorted"
        );

        assert_eq!(
            idx_arr.len(),
            data_arr.len(),
            "Indices must match data length"
        );
    }

    impl Table1D {
        pub fn validate(self) -> Self {
            validate(&self.x.1, &self.data);

            self
        }
    }

    impl Table2D {
        pub fn validate(self) -> Self {
            validate(&self.x.1, &self.data);

            for y_data in self.data.iter() {
                validate(&self.y.1, y_data);
            }

            self
        }
    }

    impl Table3D {
        pub fn validate(self) -> Self {
            validate(&self.x.1, &self.data);

            for y_data in self.data.iter() {
                validate(&self.y.1, y_data);

                for z_data in y_data.iter() {
                    validate(&self.z.1, z_data);
                }
            }

            self
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        #[should_panic(expected = "Indices must be sorted")]
        fn not_sorted() {
            Table1D::new(
                (StateVariable::Time, [0., 0.]),
                [10., 20.],
                Interpolator::default(),
            );
        }

        #[test]
        #[should_panic(expected = "Indices must match data length")]
        fn invalid_length() {
            Table1D {
                x: (StateVariable::Time, [0., 1.].into()),
                data: [10., 20., 30.].into(),
                interpolator: Interpolator::default(),
            }
            .validate();
        }

        #[test]
        #[should_panic(expected = "Indices must match data length")]
        fn invalid_length_2d_x() {
            Table2D {
                x: (StateVariable::Time, [0., 1.].into()),
                y: (StateVariable::Time, [0., 1.].into()),
                data: [[10., 20.].into(), [10., 20.].into(), [10., 20.].into()].into(),
                interpolator: Interpolator::default(),
            }
            .validate();
        }

        #[test]
        #[should_panic(expected = "Indices must match data length")]
        fn invalid_length_2d_y() {
            Table2D {
                x: (StateVariable::Time, [0., 1.].into()),
                y: (StateVariable::Time, [0., 1.].into()),
                data: [[10., 20.].into(), [10., 20., 30.].into()].into(),
                interpolator: Interpolator::default(),
            }
            .validate();
        }

        #[test]
        #[should_panic(expected = "Indices must match data length")]
        fn invalid_length_3d_z() {
            Table3D {
                x: (StateVariable::Time, [0.].into()),
                y: (StateVariable::Time, [0., 1.].into()),
                z: (StateVariable::Time, [0., 1.].into()),
                data: [[[10., 20., 30.].into(), [10., 20.].into()].into()].into(),
                interpolator: Interpolator::default(),
            }
            .validate();
        }
    }
}
