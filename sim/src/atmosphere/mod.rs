// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 22.11.23
// Last modified by Tibor Völcker on 05.03.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

pub mod standard_atmosphere_1962;

use derive_more::{Deref, DerefMut};

use nalgebra::Vector3;

use crate::planet::EnvState as PlanetState;

#[derive(Default)]
pub struct Atmosphere {
    wind: Vector3<f64>,
    model: AtmosphereModel,
}

#[derive(Default)]
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

#[derive(Default, Deref, DerefMut)]
pub struct State {
    #[deref]
    #[deref_mut]
    child_state: PlanetState,
    pub atmos_rel_velocity: Vector3<f64>,
    pub temperature: f64,
    pub pressure: f64,
    pub density: f64,
    pub speed_of_sound: f64,
    pub mach_number: f64,
    pub dynamic_pressure: f64,
}

impl Atmosphere {
    pub fn environment(&self, state: PlanetState) -> State {
        let speed_of_sound = self.speed_of_sound(&state);
        let density = self.density(&state);
        let atmos_rel_velocity = self.atmos_rel_velocity(&state);

        State {
            atmos_rel_velocity,
            temperature: self.temperature(&state),
            pressure: self.pressure(&state),
            density,
            speed_of_sound,
            mach_number: atmos_rel_velocity.norm() / speed_of_sound,
            dynamic_pressure: 0.5 * density * atmos_rel_velocity.norm().powi(2),
            child_state: state,
        }
    }
}

impl Atmosphere {
    pub fn temperature(&self, state: &PlanetState) -> f64 {
        match self.model {
            AtmosphereModel::StandardAtmosphere1962 => {
                standard_atmosphere_1962::temperature(state.geopotential_altitude)
            }
            AtmosphereModel::NoAtmosphere => 0.,
        }
    }

    pub fn pressure(&self, state: &PlanetState) -> f64 {
        match self.model {
            AtmosphereModel::StandardAtmosphere1962 => {
                standard_atmosphere_1962::pressure(state.geopotential_altitude)
            }
            AtmosphereModel::NoAtmosphere => 0.,
        }
    }

    pub fn density(&self, state: &PlanetState) -> f64 {
        match self.model {
            AtmosphereModel::StandardAtmosphere1962 => {
                standard_atmosphere_1962::density(state.geopotential_altitude)
            }
            AtmosphereModel::NoAtmosphere => 0.,
        }
    }

    pub fn speed_of_sound(&self, state: &PlanetState) -> f64 {
        match self.model {
            AtmosphereModel::StandardAtmosphere1962 => {
                standard_atmosphere_1962::speed_of_sound(state.geopotential_altitude)
            }
            AtmosphereModel::NoAtmosphere => 0.,
        }
    }

    pub fn atmos_rel_velocity(&self, state: &PlanetState) -> Vector3<f64> {
        state.rel_velocity - self.wind
    }
}
