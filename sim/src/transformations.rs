// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 17.12.23
// Last modified by Tibor Völcker on 22.05.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

//! Defines some helper function to create the transformations between the
//! coordinate frames.

use nalgebra::{matrix, Rotation3};

/// Calculates the transformation matrix from inertial frame to the launch frame.
///
/// Uses the geocentric latitude, longitude and azimuth in rad.
pub fn inertial_to_launch(lat: f64, long: f64, az: f64) -> Rotation3<f64> {
    Rotation3::from_matrix(&matrix![
                                       lat.cos()*long.cos(),                                lat.cos()*long.sin(),           lat.sin();
        lat.sin()*long.cos()*az.sin() - az.cos()*long.sin(), az.cos()*long.cos() + az.sin()*lat.sin()*long.sin(), -az.sin()*lat.cos();
       -az.sin()*long.sin() - az.cos()*lat.sin()*long.cos(), az.sin()*long.cos() - az.cos()*lat.sin()*long.sin(),  az.cos()*lat.cos()
    ])
}

/// Calculates the transformation matrix from launch frame to body frame.
///
/// Uses the roll, yaw and pitch angles in rad.
pub fn launch_to_body(roll: f64, yaw: f64, pitch: f64) -> Rotation3<f64> {
    Rotation3::from_matrix(&matrix![
        yaw.cos()*pitch.cos(), roll.cos()*yaw.sin()*pitch.cos() + roll.sin()*pitch.sin(), roll.sin()*yaw.sin()*pitch.cos() - roll.cos()*pitch.sin();
                   -yaw.sin(),                                      roll.cos()*yaw.cos(),                                      roll.sin()*yaw.cos();
        yaw.cos()*pitch.sin(), roll.cos()*yaw.sin()*pitch.sin() - roll.sin()*pitch.cos(), roll.sin()*yaw.sin()*pitch.sin() + roll.cos()*pitch.cos()
    ])
}

/// Calculates the transformation matrix from launch frame to body frame.
///
/// Uses the launch geocentric latitude, longitude and azimuth in rad, as well as the
/// euler angles in rad in the order: Roll, Yaw, Pitch.
///
/// This function calls [`launch_to_body`] and [`inertial_to_launch`].
pub fn inertial_to_body(launch: [f64; 3], euler_angles: [f64; 3]) -> Rotation3<f64> {
    launch_to_body(euler_angles[0], euler_angles[1], euler_angles[2])
        * inertial_to_launch(launch[0], launch[1], launch[2])
}

/// Calculates the transformation matrix from inertial frame to planet relative
/// frame.
///
/// Uses the current simulation time in sec and the planet's rotation rate in
/// rad/sec.
pub fn inertial_to_planet(time: f64, rotation_rate: f64) -> Rotation3<f64> {
    Rotation3::from_matrix(&matrix![
        (rotation_rate*time).cos(), (rotation_rate*time).sin(), 0.;
       -(rotation_rate*time).sin(), (rotation_rate*time).cos(), 0.;
                                0.,                         0., 1.
    ])
}
