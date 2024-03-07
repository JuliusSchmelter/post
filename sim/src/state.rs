// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 26.01.24
// Last modified by Tibor Völcker on 06.03.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::{vector, SVector, Vector2, Vector3};

pub use crate::vehicle::State;

#[derive(Debug, Default, Clone)]
pub struct PrimaryState {
    pub time: f64,
    pub time_since_event: f64,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub mass: f64,
}

impl PrimaryState {
    pub fn from_vec(time: Vector2<f64>, state: SVector<f64, 7>) -> Self {
        Self {
            time: time[0],
            time_since_event: time[1],
            position: vector![state[0], state[1], state[2]],
            velocity: vector![state[3], state[4], state[5]],
            mass: state[6],
        }
    }
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
