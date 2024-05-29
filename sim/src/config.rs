// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 21.04.24
// Last modified by Tibor Völcker on 29.05.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)

//! Defines the [`PhaseConfig`] which can deserialize the configuration file.
//! It holds the configuration for each phase and can be used to build it.
//!
//! Most fields are declared as `Option`s, as the values of the previous phase
//! are used if they are not declared.

use crate::state::StateVariable;
use crate::utils::Table;
use crate::vehicle::Engine;
use nalgebra::Vector3;
use serde::Deserialize;

/// Configurations regarding the [`crate::planet::Planet`].
#[derive(Debug, Default, Deserialize)]
#[serde(deny_unknown_fields)]
#[serde(rename_all = "snake_case")]
pub enum PlanetConfig {
    /// Spherical earth model
    /// See [`crate::planet`] for more information.
    #[default]
    Spherical,
    /// 1960 Fisher earth model (includes up to J2)
    /// See [`crate::planet`] for more information.
    Fisher1960,
    /// Smithsonian earth model (includes up to J4)
    /// See [`crate::planet`] for more information.
    Smithsonian,
    /// Use a custom implementation.
    /// See [`crate::planet`] for more information.
    Custom {
        equatorial_radius: f64,
        polar_radius: f64,
        gravitational_parameters: [f64; 4],
        rotation_rate: f64,
    },
}

/// Configurations regarding the [`crate::atmosphere::Atmosphere`].
/// The fields are `Option`s, as the values of the previous phase are used if
/// they are not declared.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AtmosphereConfig {
    /// Whether the atmosphere model is enabled.
    pub enabled: Option<bool>,
    /// Static wind vector in inertial frame in m/s.
    pub wind: Option<Vector3<f64>>,
}

/// Configurations regarding the initialization.
/// This will define the starting position and velocity of the vehicle, as well
/// as the launch frame.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InitConfig {
    /// Geodetic latitude in °.
    pub latitude: f64,
    /// Longitude latitude in °.
    pub longitude: f64,
    /// Orientation of the launch frame in ° (changes pitch direction).
    pub azimuth: f64,
    /// Altitude above the surface in m to initialize the vehicle.
    /// The launch frame is not affected.
    pub altitude: f64,
}

/// Configurations regarding the [`crate::vehicle::Vehicle`].
/// The fields are `Option`s, as the values of the previous phase are used if
/// they are not declared.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VehicleConfig {
    /// Mass of the vehicle without propellant in kg.
    pub structure_mass: Option<f64>,
    /// Initial mass of the propellant in kg.
    pub propellant_mass: Option<f64>,
    /// Reference area of the vehicle in m^2. Used for aerodynamic calculations.
    pub reference_area: Option<f64>,
    /// Table used to calculate the drag coefficients.
    pub drag_coeff: Option<Table>,
    /// Table used to calculate the lift coefficients.
    pub lift_coeff: Option<Table>,
    /// Table used to calculate the side-force coefficients.
    pub side_force_coeff: Option<Table>,
    /// Engines of the vehicle.
    /// Either all engines or no engines can be changed. To disable thrust,
    /// remove the engines by setting them to `[]`.
    pub engines: Option<Vec<Engine>>,
}

/// Configuration regarding the [`crate::steering::Steering`].
///
/// The orientation is calculated with cubic polynomials using 4 coefficients
/// and a chosen state variables.
/// The coefficients are stored in ascending order: c1*y + c2*y^2 + c3*y^3
///
/// The fields are `Option`s, as the values of the previous phase are used if
/// they are not declared.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SteeringConfig {
    /// State variable (unit X) and coefficients in °/X for the roll axis.
    pub roll: Option<(StateVariable, [f64; 3])>,
    /// State variable (unit X) and coefficients in °/X for the yaw axis.
    pub yaw: Option<(StateVariable, [f64; 3])>,
    /// State variable (unit X) and coefficients in °/X for the pitch axis.
    pub pitch: Option<(StateVariable, [f64; 3])>,
}

/// Configuration of the [`crate::phase::Phase`].
/// The fields are `Option`s, as the values of the previous phase are used if
/// they are not declared.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PhaseConfig {
    /// Initialization configuration.
    pub init: Option<InitConfig>,
    /// Vehicle configuration.
    pub vehicle: Option<VehicleConfig>,
    /// Steering configuration.
    pub steering: Option<SteeringConfig>,
    /// Planet configuration.
    pub planet_model: Option<PlanetConfig>,
    /// Atmosphere configuration.
    pub atmosphere: Option<AtmosphereConfig>,
    /// Maximum allowed acceleration in m/s^2.
    pub max_acceleration: Option<f64>,
    /// Default integrator step size in sec.
    pub stepsize: Option<f64>,
    /// The variable and its target value to end the phase.
    pub end_criterion: Option<(StateVariable, f64)>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deserialize_example() {
        let str = include_str!("../../utils/example.json");

        serde_json::from_str::<Vec<PhaseConfig>>(str).unwrap();
    }
}
