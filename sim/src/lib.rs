// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 12.11.23
// Last modified by Tibor Völcker on 23.02.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

// allow dead code for now, as it's still WIP
#![allow(dead_code)]

pub mod example_data;
pub mod integration;
mod planet;
mod state;
mod transformations;
pub mod vehicle;

pub use integration::Integrator;
use nalgebra::{vector, Vector3};
pub use planet::{Atmosphere, Planet, EARTH_FISHER_1960, EARTH_SMITHSONIAN, EARTH_SPHERICAL};
use state::{PrimaryState, State};
pub use vehicle::Vehicle;

use transformations::Transformations;

pub struct Simulation {
    pub state: PrimaryState,
    vehicle: Vehicle,
    planet: Planet,
    transformations: Transformations,
    integrator: Integrator,
    stepsize: f64,
}

impl Simulation {
    pub fn new(vehicle: Vehicle, planet: Planet, stepsize: f64) -> Self {
        Simulation {
            state: PrimaryState::new(),
            vehicle,
            planet,
            transformations: Transformations::new(),
            integrator: Integrator::RK4,
            stepsize,
        }
    }

    fn system(&self, state: &PrimaryState) -> State {
        let state = self.planet.environment(state);

        let attitude = self.vehicle.steer(state.time);

        let ib = self.transformations.inertial_to_body(
            attitude.x.to_radians(),
            attitude.y.to_radians(),
            attitude.z.to_radians(),
        );
        let state = state.add_attitude(attitude, ib);

        let pressure = self.planet.pressure(state.position);
        let rel_velocity_inertial = self
            .planet
            .atmos_rel_velocity(state.position, state.velocity);
        let rel_velocity_body = state
            .inertial_to_body
            .transform_vector(&rel_velocity_inertial);
        let alpha = self.planet.alpha(rel_velocity_body);
        let mach = self.planet.mach_number(state.position, state.velocity);
        let dynamic_pressure = self.planet.dynamic_pressure(state.position, state.velocity);
        let state = state.add_env(pressure, alpha, mach, dynamic_pressure);

        let aero = self.vehicle.aero_force(alpha, mach, dynamic_pressure);

        let throttle = self.vehicle.auto_throttle(state.mass, pressure, aero);
        let thrust = self.vehicle.thrust_force(throttle, pressure);
        let massflow = self.vehicle.massflow(throttle);

        let body_acc = (aero + thrust) / state.mass;

        // Intersection would require negative thrust
        if body_acc.norm() > self.vehicle.max_acceleration * 1.001 || throttle.is_nan() {
            panic!("Could not stay in max. acceleration (check aero forces)")
        }

        let gravity = self.planet.gravity(state.position);

        state.add_differentials(
            state.body_to_inertial.transform_vector(&body_acc) + gravity,
            massflow,
        )
    }

    pub fn step(&mut self) -> &PrimaryState {
        self.state = self
            .integrator
            .step(|state| self.system(state), &self.state, self.stepsize);
        &self.state
    }
}

impl Simulation {
    pub fn init_inertial(&mut self, position: Vector3<f64>, velocity: Vector3<f64>) {
        self.state.position = position;
        self.state.velocity = velocity;
    }

    pub fn init_geodetic(&mut self, latitude: f64, longitude: f64, azimuth: f64) {
        let (lat, long, az) = (
            latitude.to_radians(),
            longitude.to_radians(),
            azimuth.to_radians(),
        );

        let k = (self.planet.equatorial_radius / self.planet.polar_radius).powi(2);

        let geocentric_lat = f64::atan(k.powi(2) * lat.tan());

        self.transformations.launch = [geocentric_lat, long, az];

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
    }
}
