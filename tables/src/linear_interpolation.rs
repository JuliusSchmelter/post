// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 06.01.24
// Last modified by Tibor Völcker on 12.01.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)

use super::*;

pub struct Linear;
impl Interpolator for Linear {}

pub type Table1D = super::Table1D<Linear>;
pub type Table2D = super::Table2D<Linear>;
pub type Table3D = super::Table3D<Linear>;

impl Table1D {
    pub fn at(&self, x: f64) -> f64 {
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
    pub fn at(&self, x: f64, y: f64) -> f64 {
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
    pub fn at(&self, x: f64, y: f64, z: f64) -> f64 {
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
    fn extrapolate_below() {
        let table = Table1D::new([2., 3., 4., 5.], [20., 30., 40., 50.]);
        assert_eq!(table.at(1.), 10.)
    }

    #[test]
    fn extrapolate_above() {
        let table = Table1D::new([2., 3., 4., 5.], [20., 30., 40., 50.]);
        assert_eq!(table.at(6.), 60.)
    }

    #[test]
    fn interpolate_included() {
        let table = Table1D::new([2., 3., 4., 5.], [20., 30., 40., 50.]);
        assert_eq!(table.at(4.), 40.)
    }

    #[test]
    fn interpolate_in_between() {
        let table = Table1D::new([2., 3., 4., 5.], [20., 30., 40., 50.]);
        assert_eq!(table.at(3.5), 35.)
    }

    #[test]
    fn interpolate() {
        let table = Table1D::new([2., 3., 4., 5.], [20., 30., 40., 50.]);
        assert_eq!(table.at(3.125), 31.25)
    }

    #[test]
    fn interpolate_2d() {
        let table = Table2D::new([1., 2.], [10., 20.], [[100., 200.], [300., 400.]]);
        assert_eq!(table.at(1.5, 15.), 250.)
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
        );
        assert_eq!(table.at(1.5, 15., 150.), 4500.)
    }
}
