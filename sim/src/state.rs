// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 26.01.24
// Last modified by Tibor Völcker on 05.03.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::{vector, SVector, Vector3};

pub use crate::vehicle::State;

#[derive(Default)]
pub struct PrimaryState {
    pub time: f64,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub mass: f64,
}

impl State {
    pub fn new() -> Self {
        let mut s = Self::default();
        s.mass = 1.;
        s
    }

    pub fn differentials(self) -> SVector<f64, 7> {
        vector![
            self.velocity[0],
            self.velocity[1],
            self.velocity[2],
            self.acceleration[0],
            self.acceleration[1],
            self.acceleration[2],
            self.massflow,
        ]
    }

    pub fn to_primary_vec(&self) -> SVector<f64, 7> {
        vector![
            self.position[0],
            self.position[1],
            self.position[2],
            self.velocity[0],
            self.velocity[1],
            self.velocity[2],
            self.mass,
        ]
    }
}

impl From<(f64, SVector<f64, 7>)> for PrimaryState {
    fn from(val: (f64, SVector<f64, 7>)) -> Self {
        Self {
            time: val.0,
            position: vector![val.1[0], val.1[1], val.1[2]],
            velocity: vector![val.1[3], val.1[4], val.1[5]],
            mass: val.1[6],
        }
    }
}
