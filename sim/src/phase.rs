// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 12.11.23
// Last modified by Tibor Völcker on 31.03.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use crate::atmosphere::Atmosphere;
use crate::integration::Integrator;
use crate::planet::Planet;
use crate::state::State;
use crate::steering::{Axis, Steering};
use crate::transformations::{inertial_to_body, inertial_to_planet};
use crate::vehicle::Vehicle;
use crate::EARTH_SPHERICAL;
use nalgebra::{vector, Vector3};

#[derive(Debug, Clone)]
pub struct Phase {
    pub state: State,
    vehicle: Vehicle,
    steering: Steering,
    planet: Planet,
    atmosphere: Atmosphere,
    integrator: Integrator,
    stepsize: f64,
    base_stepsize: f64,
    end_criterion: fn(&State) -> f64,
    pub ended: bool,
}

impl Phase {
    fn system(&self, mut state: State) -> State {
        // Order of calculation is important, as they are dependent on each other
        // Faulty order will not raise warnings!

        // Additional primary state values
        let inertial_to_planet = inertial_to_planet(state.time, self.planet.rotation_rate);
        state.position_planet = inertial_to_planet.transform_vector(&state.position);
        state.velocity_planet = self.planet.velocity_planet(state.position, state.velocity);
        state.altitude = self.planet.altitude(state.position);
        state.altitude_geopotential = self.planet.geopotational_altitude(state.position);
        state.velocity_planet = self.planet.velocity_planet(state.position, state.velocity);

        // Gravity acceleration
        state.gravity_acceleration = self.planet.gravity(state.position);

        // Atmospheric data
        state.velocity_atmosphere = self.atmosphere.velocity_atmosphere(&state);
        state.temperature = self.atmosphere.temperature(&state);
        state.pressure = self.atmosphere.pressure(&state);
        state.density = self.atmosphere.density(&state);
        state.mach_number = self.atmosphere.mach_number(&state);
        state.dynamic_pressure = self.atmosphere.dynamic_pressure(&state);

        // Attitude
        state.euler_angles = self.steering.euler_angles(&state);
        let inertial_to_body = inertial_to_body(self.planet.launch, state.euler_angles);

        // Aerodynamic acceleration
        state.alpha = self
            .vehicle
            .alpha(inertial_to_body.transform_vector(&state.velocity_atmosphere));
        state.aero_force_body = self.vehicle.aero_force(&state);

        // Thrust acceleration
        state.propellant_mass = state.mass - self.vehicle.mass;
        state.throttle =
            self.vehicle
                .auto_throttle(state.mass, state.pressure, state.aero_force_body);
            state.thrust_force_body = self.vehicle.thrust_force(state.throttle, state.pressure);
            state.massflow = self.vehicle.massflow(state.throttle);

        // Vehicle acceleration
        state.vehicle_acceleration_body =
            (state.aero_force_body + state.thrust_force_body) / state.mass;
        if state.vehicle_acceleration_body.norm() > self.vehicle.max_acceleration * 1.001
            || state.throttle.is_nan()
        {
            // Intersection would require negative thrust
            panic!("Could not stay in max. acceleration (check aero forces)")
        }

        // Acceleration
        state.acceleration = inertial_to_body
            .transpose()
            .transform_vector(&state.vehicle_acceleration_body)
            + state.gravity_acceleration;

        state
    }

    pub fn step(&mut self) {
        if self.ended {
            panic!("Phase already has ended")
        }

        let state = self
            .integrator
            .step(|state| self.system(state), &self.state, self.stepsize);

        if (self.end_criterion)(&state) < 0. {
            // The stepsize was too big. Try again with half the stepsize.
            self.stepsize /= 2.;

            // Stop after 10 tries
            if self.base_stepsize / self.stepsize > 2_f64.powi(50) {
                panic!("Could not reach end condition")
            }

            return self.step();
        } else if (self.end_criterion)(&state) < 1e-3 {
            // We found a good last stepsize. Phase has ended.
            self.ended = true;
        }

        self.state = state;
    }

    pub fn run(&mut self) {
        // Calculate initial full state
        self.state = self.system(self.state.clone());

        while !self.ended {
            self.step();
            println!(
                "Time: {:.0}\nPosition: {:.0}\nVelocity: {:.0}\nAltitude: {:.0}\n",
                self.state.time,
                self.state.position_planet,
                self.state.velocity,
                self.state.altitude
            );
        }
    }
}

impl Default for Phase {
    fn default() -> Self {
        Self {
            state: State::new(),
            vehicle: Vehicle::default(),
            steering: Steering::new(),
            planet: EARTH_SPHERICAL,
            atmosphere: Atmosphere::new(),
            integrator: Integrator::RK4,
            stepsize: 1.,
            base_stepsize: 1.,
            end_criterion: |_| 0.,
            ended: false,
        }
    }
}
impl Phase {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn reset(&mut self) -> &mut Self {
        self.state.time_since_event = 0.;
        self.stepsize = self.base_stepsize;
        self.init_steering(self.state.euler_angles);
        self.ended = false;
        self
    }

    pub fn update_planet(&mut self, planet: Planet) -> &mut Self {
        self.planet = planet;
        self
    }

    pub fn add_atmosphere(&mut self) -> &mut Self {
        self.atmosphere.add_atmosphere();
        self
    }

    pub fn add_vehicle(&mut self, vehicle: Vehicle) -> &mut Self {
        self.vehicle = vehicle;
        self
    }

    pub fn limit_acceleration(&mut self, max_acceleration: f64) -> &mut Self {
        self.vehicle.max_acceleration = max_acceleration;
        self
    }

    pub fn init_steering(&mut self, euler_anges: [f64; 3]) -> &mut Self {
        self.steering.init(euler_anges);
        self
    }

    pub fn update_steering(&mut self, axis: Axis, coeffs: [f64; 3]) -> &mut Self {
        self.steering.update_steering(axis, coeffs);
        self
    }

    pub fn set_mass(&mut self, mass: f64) -> &mut Self {
        self.state.mass = mass;
        self
    }

    pub fn update_mass(&mut self, delta_mass: f64) -> &mut Self {
        self.state.mass += delta_mass;
        self
    }

    pub fn set_time(&mut self, time: f64) -> &mut Self {
        self.state.time = time;
        self
    }

    pub fn set_stepsize(&mut self, stepsize: f64) -> &mut Self {
        self.stepsize = stepsize;
        self.base_stepsize = stepsize;
        self
    }

    pub fn update_termination(&mut self, end_criterion: fn(&State) -> f64) -> &mut Self {
        self.end_criterion = end_criterion;
        self
    }

    pub fn init_inertial(&mut self, position: Vector3<f64>, velocity: Vector3<f64>) -> &mut Self {
        self.state.position = position;
        self.state.velocity = velocity;
        self
    }

    pub fn init_launch(&mut self, latitude: f64, longitude: f64, azimuth: f64) -> &mut Self {
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
            .velocity_planet(self.state.position, Vector3::zeros());

        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::assert_almost_eq_rel;
    use crate::example_data::{DATA_POINTS, VEHICLES};

    #[test]
    fn phase_1() {
        let vehicle = VEHICLES[0].clone();
        let mut phase = Phase::new();
        phase
            .add_vehicle(vehicle)
            .add_atmosphere()
            .init_launch(28.5, 279.4, 90.)
            .set_mass(DATA_POINTS[0].mass)
            .set_stepsize(5.)
            .update_termination(|s| 15. - s.time);

        assert_almost_eq_rel!(vec phase.state.position, DATA_POINTS[0].position, 0.001);
        assert_almost_eq_rel!(vec phase.state.velocity, DATA_POINTS[0].velocity, 0.001);

        phase.run();

        assert_eq!(phase.state.time, DATA_POINTS[1].time);
        assert_almost_eq_rel!(phase.state.mass, DATA_POINTS[1].mass, 0.001);
        assert_almost_eq_rel!(vec phase.state.position, DATA_POINTS[1].position, 0.001);
        assert_almost_eq_rel!(vec phase.state.velocity, DATA_POINTS[1].velocity, 0.001);
    }

    #[test]
    fn phase_11() {
        let vehicle = VEHICLES[1].clone();
        let mut phase = Phase::new();
        phase
            .add_vehicle(vehicle)
            .add_atmosphere()
            .init_launch(28.5, 279.4, 90.)
            .init_inertial(DATA_POINTS[2].position, DATA_POINTS[2].velocity)
            .init_steering(DATA_POINTS[2].euler_angles)
            .set_time(4.37456932e2)
            .set_stepsize(20.)
            .set_mass(DATA_POINTS[2].mass)
            .update_steering(Axis::Pitch, [DATA_POINTS[2].steering_coeffs[1], 0., 0.])
            .update_termination(|s| s.propellant_mass);

        println!(
            "Time: {:.0}\nPosition: {:.0}\nVelocity: {:.0}",
            phase.state.time, phase.state.position, phase.state.velocity
        );
        phase.run();

        // automatic thrust shutdown distorts shutdown time a bit
        assert_almost_eq_rel!(phase.state.time, DATA_POINTS[3].time, 0.006);
        assert_almost_eq_rel!(
            phase.state.time_since_event,
            DATA_POINTS[3].time_since_event,
            0.006
        );
        assert_almost_eq_rel!(phase.state.mass, DATA_POINTS[3].mass, 0.001);
        assert_almost_eq_rel!(vec phase.state.position, DATA_POINTS[3].position, 0.001);
        assert_almost_eq_rel!(vec phase.state.velocity, DATA_POINTS[3].velocity, 0.001);
    }
}
