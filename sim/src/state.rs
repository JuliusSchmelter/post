// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 26.01.24
// Last modified by Tibor Völcker on 12.03.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::{vector, Rotation3, SVector, Vector2, Vector3};

pub use crate::vehicle::State;

#[derive(Debug, Clone)]
pub struct PrimaryState {
    pub time: f64,
    pub time_since_event: f64,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub mass: f64,
}

impl Default for PrimaryState {
    fn default() -> Self {
        Self {
            time: 0.,
            time_since_event: 0.,
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            mass: 1.,
        }
    }
}

impl PrimaryState {
    pub fn from_vec(time: Vector2<f64>, state: SVector<f64, 7>) -> Self {
        Self {
            time: time[0],
            time_since_event: time[1],
            position: vector![state[0], state[1], state[2]],
            velocity: vector![state[3], state[4], state[5]],
            mass: state[6],
        }
    }
}

impl State {
    pub fn new() -> Self {
        Self::default()
    }

    #[allow(clippy::too_many_arguments)]
    pub fn from_values(
        time: f64,
        time_since_event: f64,
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        mass: f64,
        position_planet: Vector3<f64>,
        inertial_to_launch: Rotation3<f64>,
        altitude: f64,
        geopotential_altitude: f64,
        rel_velocity: Vector3<f64>,
        atmos_rel_velocity: Vector3<f64>,
        temperature: f64,
        pressure: f64,
        density: f64,
        mach_number: f64,
        dynamic_pressure: f64,
        euler_angles: [f64; 3],
        inertial_to_body: Rotation3<f64>,
        body_to_inertial: Rotation3<f64>,
        gravity_acceleration: Vector3<f64>,
        acceleration: Vector3<f64>,
        propellant_mass: f64,
        massflow: f64,
        vehicle_acceleration: Vector3<f64>,
        throttle: f64,
        thrust_force: Vector3<f64>,
        aero_force: Vector3<f64>,
        alpha: f64,
    ) -> Self {
        let mut s = Self::new();
        s.time = time;
        s.time_since_event = time_since_event;
        s.position = position;
        s.velocity = velocity;
        s.mass = mass;
        s.position_planet = position_planet;
        s.inertial_to_launch = inertial_to_launch;
        s.altitude = altitude;
        s.geopotential_altitude = geopotential_altitude;
        s.rel_velocity = rel_velocity;
        s.atmos_rel_velocity = atmos_rel_velocity;
        s.temperature = temperature;
        s.pressure = pressure;
        s.density = density;
        s.mach_number = mach_number;
        s.dynamic_pressure = dynamic_pressure;
        s.euler_angles = euler_angles;
        s.inertial_to_body = inertial_to_body;
        s.body_to_inertial = body_to_inertial;
        s.gravity_acceleration = gravity_acceleration;
        s.acceleration = acceleration;
        s.propellant_mass = propellant_mass;
        s.massflow = massflow;
        s.vehicle_acceleration = vehicle_acceleration;
        s.throttle = throttle;
        s.thrust_force = thrust_force;
        s.aero_force = aero_force;
        s.alpha = alpha;
        s
    }

    pub fn differentials(self) -> SVector<f64, 7> {
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
}
