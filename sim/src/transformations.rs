// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 17.12.23
// Last modified by Tibor Völcker on 23.02.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::{matrix, Rotation3};

pub fn inertial_to_launch(lat: f64, long: f64, az: f64) -> Rotation3<f64> {
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

pub fn inertial_to_body(
    lat: f64,
    long: f64,
    az: f64,
    roll: f64,
    yaw: f64,
    pitch: f64,
) -> Rotation3<f64> {
    launch_to_body(roll, yaw, pitch) * inertial_to_launch(lat, long, az)
}
