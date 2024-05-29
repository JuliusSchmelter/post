// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 12.11.23
// Last modified by Tibor Völcker on 29.05.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

//! Defines the [`Phase`] struct. The phase represents a section of the
//! trajectory between two events. Each phase is initialized with
//! configuration parameters and then integrated until the next event.
//!
//! The main logic of the equations of motion is implemented here.

use crate::atmosphere::Atmosphere;
use crate::config::{InitConfig, PhaseConfig};
use crate::integration::Integrator;
use crate::planet::Planet;
use crate::state::{State, StateVariable};
use crate::steering::Steering;
use crate::transformations::{inertial_to_body, inertial_to_planet};
use crate::vehicle::Vehicle;
use nalgebra::{vector, Vector3};

/// Represents a phase.
#[derive(Debug, Clone)]
pub struct Phase {
    /// The current state.
    pub state: State,
    /// The vehicle which is simulated.
    vehicle: Vehicle,
    /// The steering of the vehicle.
    steering: Steering,
    /// The atmosphere of the planet.
    atmosphere: Atmosphere,
    /// The attracting planet.
    planet: Planet,
    /// Defines the launch frame. The array consists of the geocentric
    /// latitude, longitude and azimuth in rad.
    launch: [f64; 3],
    /// The integrator used to integrate the equations of motion.
    integrator: Integrator,
    /// The specified maximum acceleration allowed in m/s^2.
    max_acceleration: f64,
    /// The step size of the current time step in sec.
    /// It is adjusted at the end of a phase to satisfy the end criterion.
    stepsize: f64,
    /// The specified time step size in sec.
    base_stepsize: f64,
    /// The variable and its target value to end the current phase.
    end_criterion: (StateVariable, f64),
    /// The number of tries to hit the target value.
    end_criterion_tries: usize,
    /// Whether the current phase has ended.
    pub ended: bool,
}

impl Phase {
    /// Represents the equations of motion. The input is a state where only the
    /// primary state (set by the integrator) is set. This function will slowly
    /// fill the state incrementally.
    ///
    /// This function will call all the other function of the [`Planet`],
    /// [`Vehicle`], [`Atmosphere`] and [`Steering`]. See the different modules
    /// for their methods.
    ///
    /// Most function require some state variables directly. But the atmosphere
    /// functions, the steering calculations and aero force calculations
    /// require the complete state. As some state variables are not set yet,
    /// this is not really safe and should be changed.
    fn system(&self, mut state: State) -> State {
        // Order of calculation is important, as they are dependent on each other
        // Faulty order will not raise warnings!

        // Additional primary state values
        let inertial_to_planet = inertial_to_planet(state.time, self.planet.rotation_rate);
        state.position_planet = inertial_to_planet.transform_vector(&state.position);
        state.velocity_planet = self.planet.velocity_planet(state.position, state.velocity);
        state.altitude = self.planet.altitude(state.position);
        state.altitude_geopotential = self.planet.geopotential_altitude(state.position);
        state.velocity_planet = self.planet.velocity_planet(state.position, state.velocity);
        state.propellant_mass = state.mass - self.vehicle.structure_mass;

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
        state.alpha = Vehicle::alpha(inertial_to_body.transform_vector(&state.velocity_atmosphere));
        state.aero_force_body = self.vehicle.aero_force(&state);

        // Thrust acceleration
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

    /// Estimates the time until the target value is reached.
    ///
    /// It does this by estimating the derivative of the cost function (meaning
    /// the difference between current and target value) using the change over
    /// the last timestep.
    fn time_to_go(&self, old_state: &State, new_state: &State) -> f64 {
        let y_t = self.end_criterion.0.get_value(old_state) - self.end_criterion.1;
        let y_t_1 = self.end_criterion.0.get_value(new_state) - self.end_criterion.1;
        let dt = self.stepsize;

        -y_t * dt / (y_t_1 - y_t)
    }

    /// Checks whether the target value was overshot
    ///
    /// It checks whether the value is increasing or decreasing over the last
    /// time step and then checks whether the current value is above or below
    /// the target value.
    fn event_is_active(&self, old_state: &State, new_state: &State) -> bool {
        if self.end_criterion.0.get_value(new_state) < self.end_criterion.0.get_value(old_state) {
            // Function is going down, so check if function < target
            self.end_criterion.0.get_value(new_state) < self.end_criterion.1
        } else {
            // Function is going up, check if function > target
            self.end_criterion.0.get_value(new_state) > self.end_criterion.1
        }
    }

    /// Does one integration step.
    ///
    /// It integrates the equations of motion for one time step. Then, it
    /// checks whether the end criterion is satisfied. If it is, the phase has
    /// ended.
    /// If the last time step overshot the target (checked with
    /// [`Phase::event_is_active`]), the time step is discarded and a new step
    /// size is calculated with [`Phase::time_to_go`].
    /// Otherwise it will simply do another time step until one of the above
    /// occurs.
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
        } else if self.event_is_active(&self.state, &state) {
            // The stepsize was too big, try again.
            if self.end_criterion_tries > 20 {
                panic!("Could not find zero crossing of event")
            }

            self.stepsize = self.time_to_go(&self.state, &state);
            self.end_criterion_tries += 1;
        } else {
            // Normal step, still more steps to go.
            self.state = state;
        }
    }

    /// Runs the Phase.
    ///
    /// This function repeatedly run [`Phase::step`] until the phase has ended.
    /// Also, it will write the current time step to output.
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
    /// The default Phase or Phase 0. The first phase will inherit from this
    /// phase and overwrite its parameters with its configuration.
    fn default() -> Self {
        Self {
            state: State::default(),
            vehicle: Vehicle::default(),
            max_acceleration: f64::INFINITY,
            steering: Steering::default(),
            planet: Planet::default(),
            launch: [0., 0., 0.],
            atmosphere: Atmosphere::default(),
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
    /// Create a new phase from the previous phase and a configuration.
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

    /// Reset the phase.
    ///
    /// It will set the time since last event to zero, initialize the steering
    /// with [`Steering::init`], resets the step size, resets the tries
    /// to reach the end criterion and sets `ended` to false.
    pub fn reset(&mut self) -> &mut Self {
        self.state.time_since_event = 0.;
        self.stepsize = self.base_stepsize;
        self.steering.init(self.state.euler_angles);
        self.ended = false;
        self.end_criterion_tries = 0;
        self
    }

    /// Initialize the phase.
    ///
    /// Only the first phase will be initialized. The function initializes the
    /// launch frame and sets the state position and velocity.
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
    use crate::config::SteeringConfig;
    use crate::example_data::DATA_POINTS;

    #[test]
    fn phase_1() {
        let str = include_str!("../../utils/example.json");

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
        let str = include_str!("../../utils/example.json");

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
        phase
            .steering
            .init([0., 0., DATA_POINTS[2].steering_coeffs[0].to_radians()]);
        phase.steering.update_with_config(&SteeringConfig {
            roll: None,
            yaw: None,
            pitch: Some((
                StateVariable::TimeSinceEvent,
                [DATA_POINTS[2].steering_coeffs[1], 0., 0.],
            )),
        });

        phase.run();

        assert_almost_eq_rel!(
            phase.state.time_since_event,
            DATA_POINTS[3].time_since_event,
            0.001
        );
        assert_almost_eq_rel!(phase.state.mass, DATA_POINTS[3].mass, 0.001);
        assert_almost_eq_rel!(vec phase.state.position, DATA_POINTS[3].position, 0.001);
        assert_almost_eq_rel!(vec phase.state.velocity, DATA_POINTS[3].velocity, 0.001);
    }
}
