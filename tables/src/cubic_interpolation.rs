// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 06.01.24
// Last modified by Tibor Völcker on 06.01.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)

#![allow(unused_variables)]
use super::*;

pub struct Cubic;
impl Interpolator for Cubic {}

pub type Table1D<const X: usize> = super::Table1D<X, Cubic>;
pub type Table2D<const X: usize, const Y: usize> = super::Table2D<X, Y, Cubic>;
pub type Table3D<const X: usize, const Y: usize, const Z: usize> = super::Table3D<X, Y, Z, Cubic>;

impl<const X: usize> Table1D<X> {
    pub fn at(&self, x: f64) -> f64 {
        todo!();
    }
}

impl<const X: usize, const Y: usize> Table2D<X, Y> {
    pub fn at(&self, x: f64, y: f64) -> f64 {
        todo!();
    }
}

impl<const X: usize, const Y: usize, const Z: usize> Table3D<X, Y, Z> {
    pub fn at(&self, x: f64, y: f64, z: f64) -> f64 {
        todo!();
    }
}
