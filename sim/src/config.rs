// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 21.04.24
// Last modified by Tibor Völcker on 08.05.24
// Copyright (c) 2024 Tibor Völcker (tiborvoelcker@hotmail.de)
use crate::{state::StateVariable, utils::Table, Engine};

use nalgebra::Vector3;
use serde::Deserialize;

#[derive(Debug, Default, Deserialize)]
#[serde(deny_unknown_fields)]
#[serde(rename_all = "snake_case")]
pub enum PlanetConfig {
    #[default]
    Spherical,
    Fisher1960,
    Smithsonian,
    Custom {
        equatorial_radius: f64,
        polar_radius: f64,
        // [mu, J_2, J_3, J_4]
        gravitational_parameters: [f64; 4],
        rotation_rate: f64,
    },
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AtmosphereConfig {
    pub enabled: Option<bool>,
    pub wind: Option<Vector3<f64>>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InitConfig {
    pub latitude: f64,
    pub longitude: f64,
    pub azimuth: f64,
    pub altitude: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VehicleConfig {
    pub structure_mass: Option<f64>,
    pub propellant_mass: Option<f64>,
    pub reference_area: Option<f64>,
    pub drag_coeff: Option<Table>,
    pub lift_coeff: Option<Table>,
    pub side_force_coeff: Option<Table>,
    pub engines: Option<Vec<Engine>>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SteeringConfig {
    pub roll: Option<(StateVariable, [f64; 3])>,
    pub yaw: Option<(StateVariable, [f64; 3])>,
    pub pitch: Option<(StateVariable, [f64; 3])>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PhaseConfig {
    pub planet_model: Option<PlanetConfig>,
    pub atmosphere: Option<AtmosphereConfig>,
    pub init: Option<InitConfig>,
    pub vehicle: Option<VehicleConfig>,
    pub max_acceleration: Option<f64>,
    pub steering: Option<SteeringConfig>,
    pub stepsize: Option<f64>,
    pub end_criterion: Option<(StateVariable, f64)>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deserialize_example() {
        let str = include_str!("utils/input.json");

        serde_json::from_str::<Vec<PhaseConfig>>(str).unwrap();
    }
}
