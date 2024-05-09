// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 26.01.24
// Last modified by Tibor Völcker on 09.05.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::{vector, SVector, Vector2, Vector3};
use serde::Deserialize;

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
}

#[derive(Debug, Default, Copy, Clone, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum StateVariable {
    #[default]
    Time,
    TimeSinceEvent,
    Position1,
    Position2,
    Position3,
    PositionNorm,
    PositionPlanet1,
    PositionPlanet2,
    PositionPlanet3,
    PositionPlanetNorm,
    Altitude,
    AltitudeGeopotential,
    Velocity1,
    Velocity2,
    Velocity3,
    VelocityNorm,
    VelocityPlanet1,
    VelocityPlanet2,
    VelocityPlanet3,
    VelocityPlanetNorm,
    VelocityAtmosphere1,
    VelocityAtmosphere2,
    VelocityAtmosphere3,
    VelocityAtmosphereNorm,
    Acceleration1,
    Acceleration2,
    Acceleration3,
    AccelerationNorm,
    ThrustForceBody1,
    ThrustForceBody2,
    ThrustForceBody3,
    ThrustForceBodyNorm,
    AeroForceBody1,
    AeroForceBody2,
    AeroForceBody3,
    AeroForceBodyNorm,
    VehicleAccelerationBody1,
    VehicleAccelerationBody2,
    VehicleAccelerationBody3,
    VehicleAccelerationBodyNorm,
    GravityAcceleration1,
    GravityAcceleration2,
    GravityAcceleration3,
    GravityAccelerationNorm,
    Mass,
    PropellantMass,
    Massflow,
    Temperature,
    Pressure,
    Density,
    MachNumber,
    DynamicPressure,
    Alpha,
    EulerAnglesRoll,
    EulerAnglesYaw,
    EulerAnglesPitch,
    Throttle,
}

impl StateVariable {
    pub fn get_value(&self, state: &State) -> f64 {
        match self {
            StateVariable::Time => state.time,
            StateVariable::TimeSinceEvent => state.time_since_event,
            StateVariable::Position1 => state.position[0],
            StateVariable::Position2 => state.position[1],
            StateVariable::Position3 => state.position[2],
            StateVariable::PositionNorm => state.position.norm(),
            StateVariable::PositionPlanet1 => state.position_planet[0],
            StateVariable::PositionPlanet2 => state.position_planet[1],
            StateVariable::PositionPlanet3 => state.position_planet[2],
            StateVariable::PositionPlanetNorm => state.position_planet.norm(),
            StateVariable::Altitude => state.altitude,
            StateVariable::AltitudeGeopotential => state.altitude_geopotential,
            StateVariable::Velocity1 => state.velocity[0],
            StateVariable::Velocity2 => state.velocity[1],
            StateVariable::Velocity3 => state.velocity[2],
            StateVariable::VelocityNorm => state.velocity.norm(),
            StateVariable::VelocityPlanet1 => state.velocity_planet[0],
            StateVariable::VelocityPlanet2 => state.velocity_planet[1],
            StateVariable::VelocityPlanet3 => state.velocity_planet[2],
            StateVariable::VelocityPlanetNorm => state.velocity_planet.norm(),
            StateVariable::VelocityAtmosphere1 => state.velocity_atmosphere[0],
            StateVariable::VelocityAtmosphere2 => state.velocity_atmosphere[1],
            StateVariable::VelocityAtmosphere3 => state.velocity_atmosphere[2],
            StateVariable::VelocityAtmosphereNorm => state.velocity_atmosphere.norm(),
            StateVariable::Acceleration1 => state.acceleration[0],
            StateVariable::Acceleration2 => state.acceleration[1],
            StateVariable::Acceleration3 => state.acceleration[2],
            StateVariable::AccelerationNorm => state.acceleration.norm(),
            StateVariable::ThrustForceBody1 => state.thrust_force_body[0],
            StateVariable::ThrustForceBody2 => state.thrust_force_body[1],
            StateVariable::ThrustForceBody3 => state.thrust_force_body[2],
            StateVariable::ThrustForceBodyNorm => state.thrust_force_body.norm(),
            StateVariable::AeroForceBody1 => state.aero_force_body[0],
            StateVariable::AeroForceBody2 => state.aero_force_body[1],
            StateVariable::AeroForceBody3 => state.aero_force_body[2],
            StateVariable::AeroForceBodyNorm => state.aero_force_body.norm(),
            StateVariable::VehicleAccelerationBody1 => state.vehicle_acceleration_body[0],
            StateVariable::VehicleAccelerationBody2 => state.vehicle_acceleration_body[1],
            StateVariable::VehicleAccelerationBody3 => state.vehicle_acceleration_body[2],
            StateVariable::VehicleAccelerationBodyNorm => state.vehicle_acceleration_body.norm(),
            StateVariable::GravityAcceleration1 => state.gravity_acceleration[0],
            StateVariable::GravityAcceleration2 => state.gravity_acceleration[1],
            StateVariable::GravityAcceleration3 => state.gravity_acceleration[2],
            StateVariable::GravityAccelerationNorm => state.gravity_acceleration.norm(),
            StateVariable::Mass => state.mass,
            StateVariable::PropellantMass => state.propellant_mass,
            StateVariable::Massflow => state.massflow,
            StateVariable::Temperature => state.temperature,
            StateVariable::Pressure => state.pressure,
            StateVariable::Density => state.density,
            StateVariable::MachNumber => state.mach_number,
            StateVariable::DynamicPressure => state.dynamic_pressure,
            StateVariable::Alpha => state.alpha,
            StateVariable::EulerAnglesRoll => state.euler_angles[0],
            StateVariable::EulerAnglesYaw => state.euler_angles[1],
            StateVariable::EulerAnglesPitch => state.euler_angles[2],
            StateVariable::Throttle => state.throttle,
        }
    }
}
