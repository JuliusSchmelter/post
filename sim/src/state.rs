// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 26.01.24
// Last modified by Tibor Völcker on 20.03.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::{vector, SVector, Vector2, Vector3};

#[derive(Debug, Default, Clone)]
pub struct State {
    pub time: f64,
    pub time_since_event: f64,
    pub position: Vector3<f64>,
    pub position_planet: Vector3<f64>,
    pub altitude: f64,
    pub altitude_geopotential: f64,
    pub velocity: Vector3<f64>,
    pub velocity_planet: Vector3<f64>,
    pub velocity_atmosphere: Vector3<f64>,
    pub acceleration: Vector3<f64>,
    pub thrust_force_body: Vector3<f64>,
    pub aero_force_body: Vector3<f64>,
    pub vehicle_acceleration_body: Vector3<f64>,
    pub gravity_acceleration: Vector3<f64>,
    pub mass: f64,
    pub propellant_mass: f64,
    pub massflow: f64,
    pub temperature: f64,
    pub pressure: f64,
    pub density: f64,
    pub mach_number: f64,
    pub dynamic_pressure: f64,
    pub alpha: f64,
    pub euler_angles: [f64; 3],
    pub throttle: f64,
}

impl State {
    pub fn new() -> Self {
        Self {
            time: 0.,
            time_since_event: 0.,
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            mass: 1.,
            ..Self::default()
        }
    }

    pub fn from_vec(time: Vector2<f64>, state: SVector<f64, 7>) -> Self {
        Self {
            time: time[0],
            time_since_event: time[1],
            position: vector![state[0], state[1], state[2]],
            velocity: vector![state[3], state[4], state[5]],
            mass: state[6],
            ..Default::default()
        }
    }

    pub fn to_differentials_vector(self) -> SVector<f64, 7> {
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

    #[allow(clippy::too_many_arguments)]
    pub fn from_values(
        time: f64,
        time_since_event: f64,
        position: Vector3<f64>,
        position_planet: Vector3<f64>,
        altitude: f64,
        altitude_geopotential: f64,
        velocity: Vector3<f64>,
        velocity_planet: Vector3<f64>,
        velocity_atmosphere: Vector3<f64>,
        acceleration: Vector3<f64>,
        thrust_force_body: Vector3<f64>,
        aero_force_body: Vector3<f64>,
        vehicle_acceleration_body: Vector3<f64>,
        gravity_acceleration: Vector3<f64>,
        mass: f64,
        propellant_mass: f64,
        massflow: f64,
        temperature: f64,
        pressure: f64,
        density: f64,
        mach_number: f64,
        dynamic_pressure: f64,
        alpha: f64,
        euler_angles: [f64; 3],
        throttle: f64,
    ) -> Self {
        Self {
            time,
            time_since_event,
            position,
            position_planet,
            altitude,
            altitude_geopotential,
            velocity,
            velocity_planet,
            velocity_atmosphere,
            acceleration,
            thrust_force_body,
            aero_force_body,
            vehicle_acceleration_body,
            gravity_acceleration,
            mass,
            propellant_mass,
            massflow,
            temperature,
            pressure,
            density,
            mach_number,
            dynamic_pressure,
            alpha,
            euler_angles,
            throttle,
        }
    }
}
