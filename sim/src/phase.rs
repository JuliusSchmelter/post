// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 12.11.23
// Last modified by Tibor Völcker on 04.03.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use crate::atmosphere::Atmosphere;
use crate::integration::Integrator;
use crate::planet::Planet;
use crate::state::{PrimaryState, State};
use crate::steering::Steering;
use crate::vehicle::Vehicle;
use nalgebra::{vector, Vector3};

pub struct Phase {
    pub state: State,
    vehicle: Vehicle,
    steering: Steering,
    planet: Planet,
    atmosphere: Atmosphere,
    integrator: Integrator,
    stepsize: f64,
    end_criterion: Box<dyn Fn(&State) -> f64>,
    pub ended: bool,
}

impl Phase {
    pub fn new(
        vehicle: Vehicle,
        planet: Planet,
        stepsize: f64,
        end_criterion: impl Fn(&State) -> f64 + 'static,
    ) -> Self {
        Phase {
            state: State::new(),
            vehicle,
            steering: Steering::new(),
            planet,
            atmosphere: Atmosphere::new(),
            integrator: Integrator::RK4,
            stepsize,
            end_criterion: Box::new(end_criterion),
            ended: false,
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

    pub fn step(&mut self) {
        if self.ended {
            panic!("Phase already has ended!")
        }

        let state = self
            .integrator
            .step(|state| self.system(state), &self.state, self.stepsize);

        if (self.end_criterion)(&state) < 0. {
            // The stepsize was too big. Try again with half the stepsize.
            self.stepsize /= 2.;
            return self.step();
        } else if (self.end_criterion)(&state) < 1e-3 {
            // We found a good last stepsize. Phase has ended.
            self.ended = true;
        }

        self.state = state;
    }
}

impl Phase {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::example_data::example_data;
    use crate::planet::EARTH_SPHERICAL;
    use utils::assert_almost_eq_rel;

    #[test]
    fn phase_1() {
        let data = example_data();

        let planet = EARTH_SPHERICAL;
        let vehicle = data[0].vehicle.clone();
        let mut sim = Phase::new(vehicle, planet, 5., |s| 15. - s.time);
        sim.add_atmosphere();

        sim.init_geodetic(28.5, 279.4, 90.);
        sim.state.mass = data[0].mass;

        assert_eq!(sim.state.time, data[0].time);
        assert_almost_eq_rel!(sim.state.mass, data[0].mass, 0.001);
        assert_almost_eq_rel!(sim.state.position[0], data[0].position[0], 0.001);
        assert_almost_eq_rel!(sim.state.position[1], data[0].position[1], 0.001);
        assert_almost_eq_rel!(sim.state.position[2], data[0].position[2], 0.001);
        assert_almost_eq_rel!(sim.state.velocity[0], data[0].velocity[0], 0.001);
        assert_almost_eq_rel!(sim.state.velocity[1], data[0].velocity[1], 0.001);
        assert_almost_eq_rel!(sim.state.velocity[2], data[0].velocity[2], 0.001);

        println!(
            "Time: {:.0}\nPosition: {:.0}\nVelocity: {:.0}",
            sim.state.time, sim.state.position, sim.state.velocity
        );
        while !sim.ended {
            sim.step();
            println!(
                "Time: {:.0}\nPosition: {:.0}\nVelocity: {:.0}",
                sim.state.time, sim.state.position, sim.state.velocity
            );
        }
        assert_eq!(sim.state.time, data[1].time);
        assert_almost_eq_rel!(sim.state.mass, data[1].mass, 0.001);
        assert_almost_eq_rel!(sim.state.position[0], data[1].position[0], 0.001);
        assert_almost_eq_rel!(sim.state.position[1], data[1].position[1], 0.001);
        assert_almost_eq_rel!(sim.state.position[2], data[1].position[2], 0.001);
        assert_almost_eq_rel!(sim.state.velocity[0], data[1].velocity[0], 0.001);
        assert_almost_eq_rel!(sim.state.velocity[1], data[1].velocity[1], 0.001);
        assert_almost_eq_rel!(sim.state.velocity[2], data[1].velocity[2], 0.001);
    }

    #[test]
    fn phase_11() {
        let data = example_data();

        let planet = EARTH_SPHERICAL;
        let vehicle = data[2].vehicle.clone();
        let vehicle_mass = data[3].mass;
        let mut sim = Phase::new(vehicle, planet, 20., move |s| s.mass - vehicle_mass);
        sim.add_steering(2, data[2].steering_coeffs);
        sim.add_atmosphere();

        sim.init_geodetic(28.5, 279.4, 90.);
        sim.init_inertial(data[2].position, data[2].velocity);
        sim.state.mass = data[2].mass;

        println!(
            "Time: {:.0}\nPosition: {:.0}\nVelocity: {:.0}",
            sim.state.time, sim.state.position, sim.state.velocity
        );
        while !sim.ended {
            sim.step();
            println!(
                "Time: {:.0}\nPosition: {:.0}\nVelocity: {:.0}",
                sim.state.time, sim.state.position, sim.state.velocity
            );
        }
        assert_almost_eq_rel!(sim.state.time, data[3].time, 0.001);
        assert_almost_eq_rel!(sim.state.mass, data[3].mass, 0.001);
        assert_almost_eq_rel!(sim.state.position[0], data[3].position[0], 0.001);
        assert_almost_eq_rel!(sim.state.position[1], data[3].position[1], 0.001);
        assert_almost_eq_rel!(sim.state.position[2], data[3].position[2], 0.001);
        assert_almost_eq_rel!(sim.state.velocity[0], data[3].velocity[0], 0.001);
        assert_almost_eq_rel!(sim.state.velocity[1], data[3].velocity[1], 0.001);
        assert_almost_eq_rel!(sim.state.velocity[2], data[3].velocity[2], 0.001);
    }
}
