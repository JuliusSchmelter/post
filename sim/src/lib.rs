// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 12.11.23
// Last modified by Tibor Völcker on 01.03.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

// allow dead code for now, as it's still WIP
#![allow(dead_code)]

mod atmosphere;
pub mod example_data;
pub mod integration;
mod planet;
mod state;
mod steering;
mod transformations;
pub mod vehicle;

pub use atmosphere::Atmosphere;
pub use integration::Integrator;
use nalgebra::{vector, Vector3};
pub use planet::{Planet, EARTH_FISHER_1960, EARTH_SMITHSONIAN, EARTH_SPHERICAL};
use state::{PrimaryState, State};
pub use steering::Steering;
pub use vehicle::Vehicle;

pub struct Simulation {
    pub state: PrimaryState,
    vehicle: Vehicle,
    steering: Steering,
    planet: Planet,
    atmosphere: Atmosphere,
    integrator: Integrator,
    stepsize: f64,
}

impl Simulation {
    pub fn new(vehicle: Vehicle, planet: Planet, stepsize: f64) -> Self {
        Simulation {
            state: PrimaryState::new(),
            vehicle,
            steering: Steering::new(),
            planet,
            atmosphere: Atmosphere::new(),
            integrator: Integrator::RK4,
            stepsize,
        }
    }

    pub fn add_steering(&mut self, idx: usize, polynomials: [f64; 4]) -> &Self {
        self.steering.add_steering(idx, polynomials);
        self
    }

    pub fn add_atmosphere(&mut self) -> &Self {
        self.atmosphere.add_atmosphere();
        self
    }

    fn system(&self, state: &PrimaryState) -> State {
        let state = self.planet.environment(state);

        let state = self.atmosphere.environment(&state);

        let state = self.steering.steering(&state);

        let state = self.planet.force(&state);

        self.vehicle.force(&state)
    }

    pub fn step(&mut self) -> &PrimaryState {
        self.state = self
            .integrator
            .step(|state| self.system(state), &self.state, self.stepsize);
        &self.state
    }
}

impl Simulation {
    pub fn init_inertial(&mut self, position: Vector3<f64>, velocity: Vector3<f64>) -> &Self {
        self.state.position = position;
        self.state.velocity = velocity;
        self
    }

    pub fn init_geodetic(&mut self, latitude: f64, longitude: f64, azimuth: f64) -> &Self {
        let (lat, long, az) = (
            latitude.to_radians(),
            longitude.to_radians(),
            azimuth.to_radians(),
        );

        let k = (self.planet.equatorial_radius / self.planet.polar_radius).powi(2);

        let geocentric_lat = f64::atan(k.powi(2) * lat.tan());

        self.planet.launch = [geocentric_lat, long, az];

        let distance_to_surface =
            self.planet.equatorial_radius / f64::sqrt(1. + (k - 1.) * geocentric_lat.sin().powi(2));

        self.state.position = distance_to_surface
            * vector![
                geocentric_lat.cos() * long.cos(),
                geocentric_lat.cos() * long.sin(),
                geocentric_lat.sin()
            ];

        self.state.velocity = -self
            .planet
            .rel_velocity(self.state.position, Vector3::zeros());

        self
    }
}
