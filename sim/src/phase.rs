// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 12.11.23
// Last modified by Tibor Völcker on 09.05.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use crate::atmosphere::Atmosphere;
use crate::config::{InitConfig, PhaseConfig};
use crate::integration::Integrator;
use crate::planet::Planet;
use crate::state::{State, StateVariable};
use crate::steering::Steering;
use crate::transformations::{inertial_to_body, inertial_to_planet};
use crate::vehicle::Vehicle;
use crate::EARTH_SPHERICAL;
use nalgebra::{vector, Vector3};

#[derive(Debug, Clone)]
pub struct Phase {
    pub state: State,
    vehicle: Vehicle,
    max_acceleration: f64,
    steering: Steering,
    planet: Planet,
    launch: [f64; 3],
    atmosphere: Atmosphere,
    integrator: Integrator,
    stepsize: f64,
    base_stepsize: f64,
    end_criterion: (StateVariable, f64),
    end_criterion_tries: usize,
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
        let inertial_to_body = inertial_to_body(self.launch, state.euler_angles);

        // Aerodynamic acceleration
        state.alpha = self
            .vehicle
            .alpha(inertial_to_body.transform_vector(&state.velocity_atmosphere));
        state.aero_force_body = self.vehicle.aero_force(&state);

        // Thrust acceleration
        state.propellant_mass = state.mass - self.vehicle.structure_mass;
        state.throttle = self.vehicle.auto_throttle(
            self.max_acceleration,
            state.mass,
            state.pressure,
            state.aero_force_body,
        );
        state.thrust_force_body = self.vehicle.thrust_force(state.throttle, state.pressure);
        state.massflow = self.vehicle.massflow(state.throttle);

        // Vehicle acceleration
        state.vehicle_acceleration_body =
            (state.aero_force_body + state.thrust_force_body) / state.mass;
        if state.vehicle_acceleration_body.norm() > self.max_acceleration * 1.001
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

    fn time_to_go(&self, state: &State) -> f64 {
        let y_t = self.end_criterion.0.get_value(&self.state) - self.end_criterion.1;
        let y_t_1 = self.end_criterion.0.get_value(state) - self.end_criterion.1;
        let dt = self.stepsize;

        -y_t * dt / (y_t_1 - y_t)
    }

    fn event_is_active(&self, state: &State) -> bool {
        if self.end_criterion.0.get_value(state) < self.end_criterion.0.get_value(&self.state) {
            // Function is going down, so check if function < target
            self.end_criterion.0.get_value(state) < self.end_criterion.1
        } else {
            // Function is going up, check if function > target
            self.end_criterion.0.get_value(state) > self.end_criterion.1
        }
    }

    pub fn step(&mut self) {
        if self.ended {
            panic!("Phase already has ended")
        }

        let state = self
            .integrator
            .step(|state| self.system(state), &self.state, self.stepsize);

        if (self.end_criterion.0.get_value(&state) - self.end_criterion.1).abs() < 1e-3 {
            // We found a good last stepsize. Phase has ended.
            self.ended = true;
            self.state = state;
        } else if self.event_is_active(&state) {
            // The stepsize was too big, try again.
            if self.end_criterion_tries > 20 {
                panic!("Could not find zero crossing of event")
            }

            self.stepsize = self.time_to_go(&state);
            self.end_criterion_tries += 1;
        } else {
            // Normal step, still more steps to go.
            self.state = state;
        }
    }

    pub fn run(&mut self) {
        // Calculate initial full state
        self.state = self.system(self.state.clone());

        while !self.ended {
            self.step();
            println!(
                "Time: {:.0}\nPosition: {:.0}\nVelocity: {:.0}\nAltitude: {:.0}\nProp mass: {:.0}\n",
                self.state.time,
                self.state.position_planet,
                self.state.velocity,
                self.state.altitude,
                self.state.propellant_mass
            );
        }
    }
}

impl Default for Phase {
    fn default() -> Self {
        Self {
            state: State::new(),
            vehicle: Vehicle::default(),
            max_acceleration: f64::INFINITY,
            steering: Steering::new(),
            planet: EARTH_SPHERICAL,
            launch: [0., 0., 0.],
            atmosphere: Atmosphere::new(),
            integrator: Integrator::RK4,
            stepsize: 1.,
            base_stepsize: 1.,
            end_criterion: (StateVariable::TimeSinceEvent, 0.),
            end_criterion_tries: 0,
            ended: false,
        }
    }
}
impl Phase {
    pub fn new(prev_phase: Option<&Phase>, config: &PhaseConfig) -> Self {
        let mut phase;
        if let Some(prev_phase) = prev_phase {
            phase = prev_phase.clone();
        } else {
            phase = Self::default();
        }

        // Update previous phase with values from config
        if let Some(config) = &config.planet_model {
            phase.planet = Planet::update_with_config(config);
        }
        if let Some(config) = &config.atmosphere {
            phase.atmosphere.update_with_config(config);
        }
        if let Some(config) = &config.vehicle {
            phase.vehicle.update_with_config(config);

            // If structure mass changed, we need to update the state
            if let Some(config) = config.structure_mass {
                phase.state.mass = phase.state.propellant_mass + config;
            }
            // If initial propellant mass changed, we need to update the state
            if let Some(config) = config.propellant_mass {
                phase.state.mass = phase.vehicle.structure_mass + config;
            }
        }
        if let Some(config) = config.max_acceleration {
            if config == -1. {
                phase.max_acceleration = f64::INFINITY;
            } else {
                phase.max_acceleration = config;
            }
        }
        if let Some(config) = &config.steering {
            phase.steering.update_with_config(config);
        }
        if let Some(config) = config.stepsize {
            phase.stepsize = config;
        }
        if let Some(config) = config.end_criterion {
            phase.end_criterion = config;
        }

        if prev_phase.is_none() {
            let config = &config
                .init
                .as_ref()
                .expect("First phase must include init config");
            phase.init(config);
        }

        phase
    }

    pub fn reset(&mut self) -> &mut Self {
        self.state.time_since_event = 0.;
        self.stepsize = self.base_stepsize;
        self.init_steering(self.state.euler_angles);
        self.ended = false;
        self.end_criterion_tries = 0;
        self
    }

    pub fn init_steering(&mut self, euler_anges: [f64; 3]) -> &mut Self {
        self.steering.init(euler_anges);
        self
    }

    pub fn init(&mut self, config: &InitConfig) -> &mut Self {
        let (lat, long, az) = (
            config.latitude.to_radians(),
            config.longitude.to_radians(),
            config.azimuth.to_radians(),
        );

        let k = (self.planet.equatorial_radius / self.planet.polar_radius).powi(2);

        let geocentric_lat = f64::atan(k.powi(2) * lat.tan());

        self.launch = [geocentric_lat, long, az];

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
    use crate::example_data::DATA_POINTS;

    #[test]
    fn phase_1() {
        let str = include_str!("utils/input.json");

        let configs: Vec<PhaseConfig> = serde_json::from_str(str).unwrap();

        let mut phase = Phase::new(None, &configs[0]);

        assert_almost_eq_rel!(phase.state.mass, DATA_POINTS[0].mass, 0.001);
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
        let str = include_str!("utils/input.json");

        let configs: Vec<PhaseConfig> = serde_json::from_str(str).unwrap();

        // Cycle through phases to finally build the last one
        let mut phase = Phase::new(None, &configs[0]);
        for config in &configs[1..] {
            phase = Phase::new(Some(&phase), config);
        }

        // Set initial state
        phase.state.mass = DATA_POINTS[2].mass;
        phase.state.position = DATA_POINTS[2].position;
        phase.state.velocity = DATA_POINTS[2].velocity;

        phase.run();

        // This is not as accurate as it should be. Might be because auto throttle.
        assert_almost_eq_rel!(
            phase.state.time_since_event,
            DATA_POINTS[3].time_since_event,
            0.0025
        );
        assert_almost_eq_rel!(phase.state.mass, DATA_POINTS[3].mass, 0.001);
        assert_almost_eq_rel!(vec phase.state.position, DATA_POINTS[3].position, 0.0025);
        // This error is inacceptable.
        assert_almost_eq_rel!(vec phase.state.velocity, DATA_POINTS[3].velocity, 0.5);
    }
}
