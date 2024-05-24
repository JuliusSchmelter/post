// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 22.11.23
// Last modified by Tibor Völcker on 24.05.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

//! Defines the [`Atmosphere`] struct, which handles all functions
//! regarding the atmosphere.

mod standard_atmosphere_1962;

use crate::config::AtmosphereConfig;
use crate::state::State;
use nalgebra::Vector3;

/// Represents the atmosphere. If the [`AtmosphereModel`] is set to
/// [`AtmosphereModel::NoAtmosphere`], no atmosphere is modeled.
///
/// It is possible that there is no atmosphere, but still wind. This is a
/// feature, not a bug.
///
/// The methods take the entire state as input, as other atmosphere models
/// might need the altitude instead of the geopotential altitude.
#[derive(Debug, Default, Clone)]
pub struct Atmosphere {
    /// Static wind vector in m/s.
    wind: Vector3<f64>,
    /// Atmosphere model used.
    model: AtmosphereModel,
}

/// Represents the different atmosphere models.
///
/// More models can be added in the future.
#[derive(Debug, Default, Clone)]
pub enum AtmosphereModel {
    /// Use no atmosphere. This sets the temperature, pressure, density, speed
    /// of sound and dynamic pressure to zero, and the mach number to infinity.
    #[default]
    NoAtmosphere,
    /// Use the 1962 U.S. Standard Atmosphere model, defined in
    /// [standard_atmosphere_1962].
    StandardAtmosphere1962,
}

impl Atmosphere {
    /// Updates itself with the new configuration parameters.
    pub fn update_with_config(&mut self, config: &AtmosphereConfig) {
        if let Some(config) = config.enabled {
            if config {
                self.model = AtmosphereModel::StandardAtmosphere1962;
            } else {
                self.model = AtmosphereModel::NoAtmosphere;
            }
        }
        if let Some(config) = config.wind {
            self.wind = config;
        }
    }
}

impl Atmosphere {
    /// Get the atmospheric temperature in K.
    ///
    /// Uses the geopotential altitude of the state.
    pub fn temperature(&self, state: &State) -> f64 {
        match self.model {
            AtmosphereModel::StandardAtmosphere1962 => {
                standard_atmosphere_1962::temperature(state.altitude_geopotential)
            }
            AtmosphereModel::NoAtmosphere => 0.,
        }
    }

    /// Get the atmospheric pressure in Pa.
    ///
    /// Uses the geopotential altitude of the state.
    pub fn pressure(&self, state: &State) -> f64 {
        match self.model {
            AtmosphereModel::StandardAtmosphere1962 => {
                standard_atmosphere_1962::pressure(state.altitude_geopotential)
            }
            AtmosphereModel::NoAtmosphere => 0.,
        }
    }

    /// Get the atmospheric density in kg/m^3.
    ///
    /// Uses the geopotential altitude of the state.
    pub fn density(&self, state: &State) -> f64 {
        match self.model {
            AtmosphereModel::StandardAtmosphere1962 => {
                standard_atmosphere_1962::density(state.altitude_geopotential)
            }
            AtmosphereModel::NoAtmosphere => 0.,
        }
    }

    /// Get the speed of sound in m/s.
    ///
    /// Uses the geopotential altitude of the state.
    fn speed_of_sound(&self, state: &State) -> f64 {
        match self.model {
            AtmosphereModel::StandardAtmosphere1962 => {
                standard_atmosphere_1962::speed_of_sound(state.altitude_geopotential)
            }
            AtmosphereModel::NoAtmosphere => 0.,
        }
    }

    /// Get the mach number.
    ///
    /// Calculates the speed of sound and uses the velocity with respect to the
    /// atmosphere of the state.
    pub fn mach_number(&self, state: &State) -> f64 {
        state.velocity_atmosphere.norm() / self.speed_of_sound(state)
    }

    /// Get the dynamic pressure in Pa.
    ///
    /// Uses the density and velocity with respect to the atmosphere of the
    /// state.
    pub fn dynamic_pressure(&self, state: &State) -> f64 {
        0.5 * state.density * state.velocity_atmosphere.norm().powi(2)
    }

    /// Calculate the velocity with respect to the atmosphere in m/s.
    ///
    /// This is the the velocity with respect to the planet minus the static
    /// wind vector.
    pub fn velocity_atmosphere(&self, state: &State) -> Vector3<f64> {
        state.velocity_planet - self.wind
    }
}

#[cfg(test)]
mod tests {
    use crate::assert_almost_eq_rel;

    use super::*;
    use crate::example_data::DATA_POINTS;

    #[test]
    fn test_environment() {
        const EPSILON: f64 = 0.001;

        let atm = Atmosphere {
            model: AtmosphereModel::StandardAtmosphere1962,
            ..Default::default()
        };

        for data_point in DATA_POINTS.iter() {
            print!("Testing {} m altitude ... ", data_point.altitude);

            let state = State {
                altitude: data_point.altitude,
                altitude_geopotential: data_point.geopotential_altitude(),
                velocity_atmosphere: data_point.velocity_planet(),
                density: data_point.density,
                ..Default::default()
            };

            assert_almost_eq_rel!(atm.temperature(&state), data_point.temperature, EPSILON);
            assert_almost_eq_rel!(atm.pressure(&state), data_point.pressure, EPSILON);
            assert_almost_eq_rel!(atm.density(&state), data_point.density, EPSILON);
            assert_almost_eq_rel!(atm.mach_number(&state), data_point.mach_number, EPSILON);
            assert_almost_eq_rel!(
                atm.dynamic_pressure(&state),
                data_point.dynamic_pressure,
                EPSILON
            );

            println!("ok");
        }
    }
}
