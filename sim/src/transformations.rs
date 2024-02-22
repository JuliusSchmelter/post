// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 17.12.23
// Last modified by Tibor Völcker on 22.02.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::{matrix, Rotation3};

#[derive(Default)]
pub struct Transformations {
    // [latitude, longitude, azimuth]
    pub launch: [f64; 3],
}

impl Transformations {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn inertial_to_launch(&self) -> Rotation3<f64> {
        let (lat, long, az) = self.launch.into();
        Rotation3::from_matrix(&matrix![
                 lat.cos()*long.cos(),                                lat.cos()*long.sin(),                                 lat.sin();
                 lat.sin()*long.cos()*az.sin() - az.cos()*long.sin(), az.cos()*long.cos() + az.sin()*lat.sin()*long.sin(), -az.sin()*lat.cos();
                -az.sin()*long.sin() - az.cos()*lat.sin()*long.cos(), az.sin()*long.cos() - az.cos()*lat.sin()*long.sin(),  az.cos()*lat.cos()
        ])
    }

    pub fn launch_to_body(roll: f64, yaw: f64, pitch: f64) -> Rotation3<f64> {
        Rotation3::from_matrix(&matrix![
             yaw.cos()*pitch.cos(), roll.cos()*yaw.sin()*pitch.cos() + roll.sin()*pitch.sin(), roll.sin()*yaw.sin()*pitch.cos() - roll.cos()*pitch.sin();
            -yaw.sin(),             roll.cos()*yaw.cos(),                                      roll.sin()*yaw.cos();
             yaw.cos()*pitch.sin(), roll.cos()*yaw.sin()*pitch.sin() - roll.sin()*pitch.cos(), roll.sin()*yaw.sin()*pitch.sin() + roll.cos()*pitch.cos()
        ])
    }

    pub fn inertial_to_body(&self, roll: f64, yaw: f64, pitch: f64) -> Rotation3<f64> {
        Self::launch_to_body(roll, yaw, pitch) * self.inertial_to_launch()
    }
}
