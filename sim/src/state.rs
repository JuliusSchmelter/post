// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 26.01.24
// Last modified by Tibor Völcker on 22.02.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::{vector, Rotation3, SVector, Vector3};

pub struct State {
    pub time: f64,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub acceleration: Vector3<f64>,
    pub mass: f64,
    pub massflow: f64,
    pub attitude: Vector3<f64>,
    pub inertial_to_body: Rotation3<f64>,
    pub body_to_inertial: Rotation3<f64>,
    pub pressure: f64,
    pub alpha: f64,
    pub mach: f64,
    pub dynamic_pressure: f64,
}

#[derive(Default)]
pub struct PrimaryState {
    pub time: f64,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub mass: f64,
}

impl PrimaryState {
    pub fn new() -> Self {
        Self {
            time: 0.,
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            mass: 1.,
        }
    }
}

pub(crate) struct SecondaryState {
    pub time: f64,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub mass: f64,
    pub attitude: Vector3<f64>,
    pub inertial_to_body: Rotation3<f64>,
    pub body_to_inertial: Rotation3<f64>,
}

pub(crate) struct TertiaryState {
    pub time: f64,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub mass: f64,
    pub attitude: Vector3<f64>,
    pub inertial_to_body: Rotation3<f64>,
    pub body_to_inertial: Rotation3<f64>,
    pub pressure: f64,
    pub alpha: f64,
    pub mach: f64,
    pub dynamic_pressure: f64,
}

impl State {
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
}

impl From<&PrimaryState> for SVector<f64, 7> {
    fn from(val: &PrimaryState) -> Self {
        vector![
            val.position[0],
            val.position[1],
            val.position[2],
            val.velocity[0],
            val.velocity[1],
            val.velocity[2],
            val.mass,
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

impl PrimaryState {
    pub(crate) fn add_attitude(
        &self,
        attitude: Vector3<f64>,
        inertial_to_body: Rotation3<f64>,
    ) -> SecondaryState {
        SecondaryState {
            time: self.time,
            position: self.position,
            velocity: self.velocity,
            mass: self.mass,
            attitude,
            inertial_to_body,
            body_to_inertial: inertial_to_body.transpose(),
        }
    }
}

impl SecondaryState {
    pub fn add_env(
        &self,
        pressure: f64,
        alpha: f64,
        mach: f64,
        dynamic_pressure: f64,
    ) -> TertiaryState {
        TertiaryState {
            time: self.time,
            position: self.position,
            velocity: self.velocity,
            mass: self.mass,
            attitude: self.attitude,
            inertial_to_body: self.inertial_to_body,
            body_to_inertial: self.body_to_inertial,
            pressure,
            alpha,
            mach,
            dynamic_pressure,
        }
    }
}

impl TertiaryState {
    pub fn add_differentials(&self, acceleration: Vector3<f64>, massflow: f64) -> State {
        State {
            time: self.time,
            position: self.position,
            velocity: self.velocity,
            acceleration,
            mass: self.mass,
            massflow,
            attitude: self.attitude,
            inertial_to_body: self.inertial_to_body,
            body_to_inertial: self.body_to_inertial,
            pressure: self.pressure,
            alpha: self.alpha,
            mach: self.mach,
            dynamic_pressure: self.dynamic_pressure,
        }
    }
}
