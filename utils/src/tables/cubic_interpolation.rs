// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 06.01.24
// Last modified by Tibor Völcker on 17.01.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)

#![allow(unused_variables)]
use super::*;

#[derive(Clone)]
pub struct Cubic;
impl Interpolator for Cubic {}

pub type Table1D = super::Table1D<Cubic>;
pub type Table2D = super::Table2D<Cubic>;
pub type Table3D = super::Table3D<Cubic>;

impl Table1D {
    pub fn at(&self, x: f64) -> f64 {
        todo!();
    }
}

impl Table2D {
    pub fn at(&self, x: f64) -> f64 {
        todo!();
    }
}

impl Table3D {
    pub fn at(&self, x: f64) -> f64 {
        todo!();
    }
}
