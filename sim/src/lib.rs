// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 04.03.23
// Last modified by Tibor Völcker on 22.05.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

//! Defines the [`Simulation`] struct which handles the simulation.
//! Each simulation is split into multiple [`Phase`]s, which is are sections of
//! the simulation between two events.
//! At each event, the configuration parameters can be changed which are loaded
//! by the [`Simulation`] struct.
//! The configuration is stored in the [`PhaseConfig`].

mod atmosphere;
mod config;
mod example_data;
mod integration;
mod phase;
mod planet;
mod state;
mod steering;
mod transformations;
mod utils;
mod vehicle;

use config::PhaseConfig;
use phase::Phase;
use state::State;
use std::{error::Error, fs::File, io::BufReader, path::Path};

/// Represents the simulation.
#[derive(Debug, Default)]
pub struct Simulation {
    /// An array of each phase configuration.
    config: Vec<PhaseConfig>,
}

impl Simulation {
    /// Creates the simulation from a filepath of the configuration file.
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self, Box<dyn Error>> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);

        let config = serde_json::from_reader(reader)?;

        Ok(Self { config })
    }

    /// Runs the simulation. It will create each phase from the previous phase
    /// and its configuration with [`Phase::new`], reset it with
    /// [`Phase::reset`] and run it with [`Phase::run`].
    pub fn run(&self) -> State {
        let mut prev_phase = None;
        let mut phase = Phase::default();
        for (i, config) in self.config.iter().enumerate() {
            println!("Starting Phase {}", i + 1);
            phase = Phase::new(prev_phase, config);
            phase.reset();

            phase.run();

            prev_phase = Some(&phase);
        }

        phase.state
    }
}

#[cfg(test)]
mod tests {
    use crate::example_data::DATA_POINTS;

    use super::*;

    #[test]
    fn full_test() {
        const TARGET_ALT: f64 = 9.25997640e4;
        const TARGET_VEL: f64 = 7.87999440e3;

        let str = include_str!("utils/input.json");

        let config: Vec<PhaseConfig> = serde_json::from_str(str).unwrap();

        let sim = Simulation { config };
        let state = sim.run();

        assert_almost_eq_rel!(state.altitude, TARGET_ALT, 0.002);
        assert_almost_eq_rel!(state.velocity.norm(), TARGET_VEL, 0.00003);
        assert_almost_eq_rel!(state.time, DATA_POINTS[3].time, 0.003);
    }
}
