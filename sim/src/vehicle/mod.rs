// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 22.11.23
// Last modified by Tibor Völcker on 06.12.23
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::{vector, Vector3};

pub struct Vehicle {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    mass: f64,
    engines: Vec<Engine>,
}

impl Vehicle {
    pub fn new(mass: f64, engines: Vec<Engine>) -> Self {
        Self {
            position: vector![0., 0., 0.],
            velocity: vector![0., 0., 0.],
            mass,
            engines,
        }
    }

    pub fn thrust(&self, pressure_atmos: f64) -> Vector3<f64> {
        return self
            .engines
            .iter()
            .map(|eng| eng.thrust(pressure_atmos))
            .sum::<Vector3<f64>>()
            / self.mass;
    }
}

pub struct Engine {
    // [pitch, yaw]
    incidence: [f64; 2],
    throttle: f64,
    thrust_vac: f64,
    exit_area: f64,
}

impl Engine {
    pub fn new(incidence: [f64; 2], thrust_vac: f64, exit_area: f64) -> Self {
        Self {
            incidence,
            throttle: 1.,
            thrust_vac,
            exit_area,
        }
    }

    fn thrust(&self, pressure_atmos: f64) -> Vector3<f64> {
        vector![
            self.incidence[1].cos() * self.incidence[0].cos(),
            self.incidence[1].sin(),
            self.incidence[1].cos() * self.incidence[0].sin()
        ] * (self.throttle * self.thrust_vac - self.exit_area * pressure_atmos)
    }
}

// TODO: Unit tests
