// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 06.01.24
// Last modified by Tibor Völcker on 26.03.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)

use crate::State;

use super::*;

impl TableTrait for f64 {
    fn at_state(&self, _state: &State) -> f64 {
        *self
    }
}

impl<T: TableTrait> TableTrait for Table<T> {
    fn at_state(&self, state: &State) -> f64 {
        // Assumptions:
        //  1. `self.x` is sorted
        //  2. `self.x.len() == self.data.len()`

        let x = self.x.0.get_value(state);

        if self.data.len() == 0 {
            // No data cannot be interpolated.
            return f64::NAN;
        }
        if self.data.len() == 1 {
            // Interplate single data point with a straight line.
            return self.data[0].at_state(state);
        }

        // Get index of upper base (index of closes bigger number)
        let idx1 = {
            let mut idx1 = self.x.1.partition_point(|val| val < &x);
            if idx1 == self.x.1.len() {
                // No bigger number. Use the last two as bases.
                idx1 -= 1;
            } else if idx1 == 0 {
                // All numbers are bigger. Use the first two as bases.
                idx1 = 1;
            }
            idx1
        };

        let x0 = self.x.1[idx1 - 1];
        let x1 = self.x.1[idx1];
        let y0 = self.data[idx1 - 1].at_state(state);
        let y1 = self.data[idx1].at_state(state);

        y0 + (x - x0) * (y1 - y0) / (x1 - x0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_table() {
        let table = Table1D::default();

        let s = State::new();
        assert!(table.at_state(&s).is_nan())
    }

    #[test]
    fn one_entry() {
        let table = Table1D::new((StateVariable::Time, [0.]), [1.34]);

        let mut s = State::new();
        assert_eq!(table.at_state(&s), 1.34);

        s.time = 999.;
        assert_eq!(table.at_state(&s), 1.34);
    }

    #[test]
    fn extrapolate_below() {
        let table = Table1D::new(
            (StateVariable::Time, [2., 3., 4., 5.]),
            [20., 30., 40., 50.],
        );

        let mut s = State::new();
        s.time = 1.;
        assert_eq!(table.at_state(&s), 10.);
    }

    #[test]
    fn extrapolate_above() {
        let table = Table1D::new(
            (StateVariable::Time, [2., 3., 4., 5.]),
            [20., 30., 40., 50.],
        );

        let mut s = State::new();
        s.time = 6.;
        assert_eq!(table.at_state(&s), 60.);
    }

    #[test]
    fn interpolate_included() {
        let table = Table1D::new(
            (StateVariable::Time, [2., 3., 4., 5.]),
            [20., 30., 40., 50.],
        );

        let mut s = State::new();
        s.time = 4.;
        assert_eq!(table.at_state(&s), 40.);
    }

    #[test]
    fn interpolate_in_between() {
        let table = Table1D::new(
            (StateVariable::Time, [2., 3., 4., 5.]),
            [20., 30., 40., 50.],
        );

        let mut s = State::new();
        s.time = 3.5;
        assert_eq!(table.at_state(&s), 35.);
    }

    #[test]
    fn interpolate() {
        let table = Table1D::new(
            (StateVariable::Time, [2., 3., 4., 5.]),
            [20., 30., 40., 50.],
        );

        let mut s = State::new();
        s.time = 3.125;
        assert_eq!(table.at_state(&s), 31.25);
    }

    #[test]
    fn empty_table_2d() {
        let table = Table2D::default();

        let s = State::new();
        assert!(table.at_state(&s).is_nan())
    }

    #[test]
    fn interpolate_2d() {
        let table = Table2D::new(
            (StateVariable::Time, [1., 2.]),
            (StateVariable::Mass, [10., 20.]),
            [[100., 200.], [300., 400.]],
        );

        let mut s = State::new();
        s.time = 1.5;
        s.mass = 15.;
        assert_eq!(table.at_state(&s), 250.);
    }

    #[test]
    fn empty_table_3d() {
        let table = Table3D::default();

        let s = State::new();
        assert!(table.at_state(&s).is_nan())
    }

    #[test]
    fn interpolate_3d() {
        let table = Table3D::new(
            (StateVariable::Time, [1., 2.]),
            (StateVariable::Mass, [10., 20.]),
            (StateVariable::Altitude, [100., 200.]),
            [
                [[1000., 2000.], [3000., 4000.]],
                [[5000., 6000.], [7000., 8000.]],
            ],
        );

        let mut s = State::new();
        s.time = 1.5;
        s.mass = 15.;
        s.altitude = 150.;
        assert_eq!(table.at_state(&s), 4500.);
    }
}
