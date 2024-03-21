// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 06.01.24
// Last modified by Tibor Völcker on 21.03.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)

use crate::State;

use super::*;

#[derive(Debug, Default, Clone)]
pub struct Linear;
impl Interpolator for Linear {}

pub type Table1D = super::Table1D<Linear>;
pub type Table2D = super::Table2D<Linear>;
pub type Table3D = super::Table3D<Linear>;

impl Table1D {
    pub fn at_state(&self, state: &State) -> f64 {
        let val = self.variable.get_value(state);
        self.at(val)
    }

    pub fn at(&self, x: f64) -> f64 {
        if self.data.len() == 0 {
            return f64::NAN;
        }
        if self.data.len() == 1 {
            return self.data[0];
        }

        let idx1 = {
            let mut idx1 = self.x.partition_point(|val| val < &x);
            if idx1 == self.x.len() {
                idx1 -= 1;
            } else if idx1 == 0 {
                idx1 = 1;
            }
            idx1
        };
        let x0 = self.x[idx1 - 1];
        let x1 = self.x[idx1];
        let y0 = self.data[idx1 - 1];
        let y1 = self.data[idx1];

        y0 + (x - x0) * (y1 - y0) / (x1 - x0)
    }
}

impl Table2D {
    pub fn at_state(&self, state: &State) -> f64 {
        if self.data.len() == 0 {
            return f64::NAN;
        }
        let x = self.variable.get_value(state);
        let y = self.data[0].variable.get_value(state);
        self.at(x, y)
    }

    pub fn at(&self, x: f64, y: f64) -> f64 {
        if self.data.len() == 0 {
            return f64::NAN;
        }
        if self.data.len() == 1 {
            return self.data[0].at(y);
        }

        let idx1 = {
            let mut idx1 = self.x.partition_point(|val| val < &x);
            if idx1 == self.x.len() {
                idx1 -= 1;
            } else if idx1 == 0 {
                idx1 = 1;
            }
            idx1
        };
        let x0 = self.x[idx1 - 1];
        let x1 = self.x[idx1];
        let y0 = self.data[idx1 - 1].at(y);
        let y1 = self.data[idx1].at(y);

        y0 + (x - x0) * (y1 - y0) / (x1 - x0)
    }
}

impl Table3D {
    pub fn at_state(&self, state: &State) -> f64 {
        if self.data.len() == 0 || self.data[0].data.len() == 0 {
            return f64::NAN;
        }
        let x = self.variable.get_value(state);
        let y = self.data[0].variable.get_value(state);
        let z = self.data[0].data[0].variable.get_value(state);
        self.at(x, y, z)
    }

    pub fn at(&self, x: f64, y: f64, z: f64) -> f64 {
        if self.data.len() == 0 {
            return f64::NAN;
        }
        if self.data.len() == 1 {
            return self.data[0].at(y, z);
        }

        let idx1 = {
            let mut idx1 = self.x.partition_point(|val| val < &x);
            if idx1 == self.x.len() {
                idx1 -= 1;
            } else if idx1 == 0 {
                idx1 = 1;
            }
            idx1
        };
        let x0 = self.x[idx1 - 1];
        let x1 = self.x[idx1];
        let y0 = self.data[idx1 - 1].at(y, z);
        let y1 = self.data[idx1].at(y, z);

        y0 + (x - x0) * (y1 - y0) / (x1 - x0)
    }
}

#[cfg(test)]
mod tests {
    use super::linear_interpolation::*;

    #[test]
    fn empty_table() {
        let table = Table1D::default();
        assert!(table.at(1.).is_nan());

        let s = State::new();
        assert!(table.at_state(&s).is_nan())
    }

    #[test]
    fn one_entry() {
        let table = Table1D::new([0.], [1.34], StateVariable::Time);
        assert_eq!(table.at(0.), 1.34);
        assert_eq!(table.at(999.), 1.34);

        let mut s = State::new();
        assert_eq!(table.at_state(&s), 1.34);

        s.time = 999.;
        assert_eq!(table.at_state(&s), 1.34);
    }

    #[test]
    #[should_panic(expected = "Data must be sorted by indexing value")]
    fn not_sorted() {
        Table1D::new([0., 0.], [10., 20.], StateVariable::Time);
    }

    #[test]
    fn extrapolate_below() {
        let table = Table1D::new([2., 3., 4., 5.], [20., 30., 40., 50.], StateVariable::Time);
        assert_eq!(table.at(1.), 10.);

        let mut s = State::new();
        s.time = 1.;
        assert_eq!(table.at_state(&s), 10.);
    }

    #[test]
    fn extrapolate_above() {
        let table = Table1D::new([2., 3., 4., 5.], [20., 30., 40., 50.], StateVariable::Time);
        assert_eq!(table.at(6.), 60.);

        let mut s = State::new();
        s.time = 6.;
        assert_eq!(table.at_state(&s), 60.);
    }

    #[test]
    fn interpolate_included() {
        let table = Table1D::new([2., 3., 4., 5.], [20., 30., 40., 50.], StateVariable::Time);
        assert_eq!(table.at(4.), 40.);

        let mut s = State::new();
        s.time = 4.;
        assert_eq!(table.at_state(&s), 40.);
    }

    #[test]
    fn interpolate_in_between() {
        let table = Table1D::new([2., 3., 4., 5.], [20., 30., 40., 50.], StateVariable::Time);
        assert_eq!(table.at(3.5), 35.);

        let mut s = State::new();
        s.time = 3.5;
        assert_eq!(table.at_state(&s), 35.);
    }

    #[test]
    fn interpolate() {
        let table = Table1D::new([2., 3., 4., 5.], [20., 30., 40., 50.], StateVariable::Time);
        assert_eq!(table.at(3.125), 31.25);

        let mut s = State::new();
        s.time = 3.125;
        assert_eq!(table.at_state(&s), 31.25);
    }

    #[test]
    fn empty_table_2d() {
        let table = Table2D::default();
        assert!(table.at(1., 2.).is_nan());

        let s = State::new();
        assert!(table.at_state(&s).is_nan())
    }

    #[test]
    fn interpolate_2d() {
        let table = Table2D::new(
            [1., 2.],
            [10., 20.],
            [[100., 200.], [300., 400.]],
            [StateVariable::Time, StateVariable::Mass],
        );
        assert_eq!(table.at(1.5, 15.), 250.);

        let mut s = State::new();
        s.time = 1.5;
        s.mass = 15.;
        assert_eq!(table.at_state(&s), 250.);
    }

    #[test]
    fn empty_table_3d() {
        let table = Table3D::default();
        assert!(table.at(1., 2., 3.).is_nan());

        let s = State::new();
        assert!(table.at_state(&s).is_nan())
    }

    #[test]
    fn interpolate_3d() {
        let table = Table3D::new(
            [1., 2.],
            [10., 20.],
            [100., 200.],
            [
                [[1000., 2000.], [3000., 4000.]],
                [[5000., 6000.], [7000., 8000.]],
            ],
            [
                StateVariable::Time,
                StateVariable::Mass,
                StateVariable::Altitude,
            ],
        );
        assert_eq!(table.at(1.5, 15., 150.), 4500.);

        let mut s = State::new();
        s.time = 1.5;
        s.mass = 15.;
        s.altitude = 150.;
        assert_eq!(table.at_state(&s), 4500.);
    }
}
