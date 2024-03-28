// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 22.11.23
// Last modified by Tibor Völcker on 28.03.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

pub mod standard_atmosphere_1962;

use nalgebra::Vector3;

use crate::State;

#[derive(Debug, Default, Clone)]
pub struct Atmosphere {
    wind: Vector3<f64>,
    model: AtmosphereModel,
}

#[derive(Debug, Default, Clone)]
pub enum AtmosphereModel {
    #[default]
    NoAtmosphere,
    StandardAtmosphere1962,
}

impl Atmosphere {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_atmosphere(&mut self) -> &Self {
        self.model = AtmosphereModel::StandardAtmosphere1962;
        self
    }
}

impl Atmosphere {
    pub fn temperature(&self, state: &State) -> f64 {
        match self.model {
            AtmosphereModel::StandardAtmosphere1962 => {
                standard_atmosphere_1962::temperature(state.altitude_geopotential)
            }
            AtmosphereModel::NoAtmosphere => 0.,
        }
    }

    pub fn pressure(&self, state: &State) -> f64 {
        match self.model {
            AtmosphereModel::StandardAtmosphere1962 => {
                standard_atmosphere_1962::pressure(state.altitude_geopotential)
            }
            AtmosphereModel::NoAtmosphere => 0.,
        }
    }

    pub fn density(&self, state: &State) -> f64 {
        match self.model {
            AtmosphereModel::StandardAtmosphere1962 => {
                standard_atmosphere_1962::density(state.altitude_geopotential)
            }
            AtmosphereModel::NoAtmosphere => 0.,
        }
    }

    fn speed_of_sound(&self, state: &State) -> f64 {
        match self.model {
            AtmosphereModel::StandardAtmosphere1962 => {
                standard_atmosphere_1962::speed_of_sound(state.altitude_geopotential)
            }
            AtmosphereModel::NoAtmosphere => 0.,
        }
    }

    pub fn mach_number(&self, state: &State) -> f64 {
        state.velocity_atmosphere.norm() / self.speed_of_sound(state)
    }

    pub fn dynamic_pressure(&self, state: &State) -> f64 {
        0.5 * state.density * state.velocity_atmosphere.norm().powi(2)
    }

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

        let mut atm = Atmosphere::new();
        atm.add_atmosphere();

        for data_point in DATA_POINTS.iter() {
            print!("Testing {} m altitude ... ", data_point.altitude);

            let state = data_point.to_state();

            assert_almost_eq_rel!(atm.temperature(&state), state.temperature, EPSILON);
            assert_almost_eq_rel!(atm.pressure(&state), state.pressure, EPSILON);
            assert_almost_eq_rel!(atm.density(&state), state.density, EPSILON);
            assert_almost_eq_rel!(atm.mach_number(&state), state.mach_number, EPSILON);
            assert_almost_eq_rel!(
                atm.dynamic_pressure(&state),
                state.dynamic_pressure,
                EPSILON
            );

            println!("ok");
        }
    }
}
