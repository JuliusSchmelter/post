// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 26.01.24
// Last modified by Tibor Völcker on 22.05.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)

//! This module defines the [`State`] used throughout the project and the
//! [`StateVariable`], which is used the retrieve a variable from the
//! [`State`].

use nalgebra::{vector, SVector, Vector2, Vector3};
use serde::Deserialize;

/// Represents the simulation state.
///
/// It is incrementally written to for each timestep.
#[derive(Debug, Default, Clone)]
pub struct State {
    /// Current simulation time in sec.
    pub time: f64,
    /// Time since last event in sec.
    pub time_since_event: f64,
    /// Inertial position in m.
    pub position: Vector3<f64>,
    /// Planet-relative position in m.
    pub position_planet: Vector3<f64>,
    /// Altitude above planet's oblate surface in m.
    pub altitude: f64,
    /// Geopotential altitude in m (used in atmospheric model).
    pub altitude_geopotential: f64,
    /// Inertial velocity in m/s.
    pub velocity: Vector3<f64>,
    /// Velocity with respect to the planet in m/s.
    pub velocity_planet: Vector3<f64>,
    /// Velocity with respect to the atmosphere in m/s.
    pub velocity_atmosphere: Vector3<f64>,
    /// Inertial acceleration in m/s^2.
    pub acceleration: Vector3<f64>,
    /// Thrust force in body frame in N.
    pub thrust_force_body: Vector3<f64>,
    /// Aerodynamic force in body frame in N.
    pub aero_force_body: Vector3<f64>,
    /// "Sensed" acceleration of the vehicle in body frame in m/s^2.
    pub vehicle_acceleration_body: Vector3<f64>,
    /// Acceleration due to gravity in m/s^2.
    pub gravity_acceleration: Vector3<f64>,
    /// Vehicle total mass in kg.
    pub mass: f64,
    /// Remaining propellant mass in kg.
    pub propellant_mass: f64,
    /// Propellant mass flow in kg/s.
    pub massflow: f64,
    /// Atmospheric temperature in K.
    pub temperature: f64,
    /// Atmospheric pressure in Pa.
    pub pressure: f64,
    /// Atmospheric density in kg/m^3.
    pub density: f64,
    /// Vehicle mach number.
    pub mach_number: f64,
    /// Dynamic pressure in Pa.
    pub dynamic_pressure: f64,
    /// Angle-of-attack in rad.
    pub alpha: f64,
    /// Euler-angles in rad in the order: Roll, Yaw, Pitch
    pub euler_angles: [f64; 3],
    /// Engine throttle setting
    pub throttle: f64,
}

impl State {
    /// Create a state from a time and state vector.
    ///
    /// The state vector only contains the primary states: position, velocity
    /// and mass. All others are set to zero.
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

    /// Return a vector of the primary state differentials: velocity,
    /// acceleration and mass flow.
    pub fn to_differentials_vector(&self) -> SVector<f64, 7> {
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

    /// Return a vector of the primary state: position, velocity and mass.
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

/// An enum for choosing a state variable.
/// Each
#[derive(Debug, Default, Copy, Clone, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum StateVariable {
    /// Simulation time
    #[default]
    Time,
    /// Time since the last event                                     
    TimeSinceEvent,
    /// Inertial position (X)                                      
    Position1,
    /// Inertial position (Y)  
    Position2,
    /// Inertial position (Z)  
    Position3,
    /// Distance from earth center                                    
    PositionNorm,
    /// Earth-relative position (X)                                     
    PositionPlanet1,
    /// Earth-relative position (Y)     
    PositionPlanet2,
    /// Earth-relative position (Z)     
    PositionPlanet3,
    /// Height above surface                                          
    Altitude,
    /// Used for the atmospheric model
    AltitudeGeopotential,
    /// Inertial velocity (X)                                             
    Velocity1,
    /// Inertial velocity (Y)  
    Velocity2,
    /// Inertial velocity (Z)  
    Velocity3,
    /// Total inertial velocity                                       
    VelocityNorm,
    /// Earth-relative velocity  (X)                                     
    VelocityPlanet1,
    /// Earth-relative velocity  (Y)     
    VelocityPlanet2,
    /// Earth-relative velocity  (Z)     
    VelocityPlanet3,
    /// Total earth-relative velocity                                 
    VelocityPlanetNorm,
    /// Atmosphere-relative velocity (X)                                  
    VelocityAtmosphere1,
    /// Atmosphere-relative velocity (Y)
    VelocityAtmosphere2,
    /// Atmosphere-relative velocity (Z)
    VelocityAtmosphere3,
    /// Total atmosphere-relative velocity                            
    VelocityAtmosphereNorm,
    /// Inertial acceleration (X)                                         
    Acceleration1,
    /// Inertial acceleration (Y)
    Acceleration2,
    /// Inertial acceleration (Z)
    Acceleration3,
    /// Total inertial acceleration                                   
    AccelerationNorm,
    /// Thrust force in body frame (X)                                    
    ThrustForceBody1,
    /// Thrust force in body frame (Y)
    ThrustForceBody2,
    /// Thrust force in body frame (Z)
    ThrustForceBody3,
    /// Total thrust force in body frame                              
    ThrustForceBodyNorm,
    /// Aerodynamic force in body framen (X)                               
    AeroForceBody1,
    /// Aerodynamic force in body framen (Y)
    AeroForceBody2,
    /// Aerodynamic force in body framen (Z)
    AeroForceBody3,
    /// Total aerodynamic force in body frame                         
    AeroForceBodyNorm,
    /// Vehicle sensed acceleration => Acceleration due to thrust and aero forces (X)                               
    VehicleAccelerationBody1,
    /// Vehicle sensed acceleration => Acceleration due to thrust and aero forces (Y)                               
    VehicleAccelerationBody2,
    /// Vehicle sensed acceleration => Acceleration due to thrust and aero forces (Z)                               
    VehicleAccelerationBody3,
    /// Total vehicle senses acceleration                             
    VehicleAccelerationBodyNorm,
    /// Acceleration due to gravity (X)                                  
    GravityAcceleration1,
    /// Acceleration due to gravity (Y)
    GravityAcceleration2,
    /// Acceleration due to gravity (Z)
    GravityAcceleration3,
    /// Total acceleration due to gravity                             
    GravityAccelerationNorm,
    /// Total vehicle mass                                            
    Mass,
    /// Propellant mass                                               
    PropellantMass,
    /// Propellant massflow                                           
    Massflow,
    /// Atmosphere temperature                                        
    Temperature,
    /// Atmosphere pressure                                           
    Pressure,
    /// Atmosphere density                                            
    Density,
    /// Vehicle mach number                                           
    MachNumber,
    /// Dynamic pressure                                              
    DynamicPressure,
    /// Angle of attack                                               
    Alpha,
    /// Roll angle with respect to launch frame                       
    EulerAnglesRoll,
    /// Yaw angle with respect to launch frame                        
    EulerAnglesYaw,
    /// Pitch angle with respect to launch frame                      
    EulerAnglesPitch,
    /// Computed auto-throttle                                        
    Throttle,
}

impl StateVariable {
    /// Retrieves the value from a state object
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
