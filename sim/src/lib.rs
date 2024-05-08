// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 04.03.23
// Last modified by Tibor Völcker on 08.05.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

// allow dead code for now, as it's still WIP
#![allow(dead_code)]

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
pub use phase::Phase;
pub use planet::{Planet, EARTH_SPHERICAL};
pub use state::State;
pub use steering::Steering;
pub use utils::constants;
pub use vehicle::{Engine, Vehicle};

#[derive(Debug, Default)]
pub struct Simulation {
    config: Vec<PhaseConfig>,
}

impl Simulation {
    pub fn new(config: Vec<PhaseConfig>) -> Self {
        Self { config }
    }

    pub fn run(&self) {
        let mut prev_phase = None;
        let mut phase;
        for (i, config) in self.config.iter().enumerate() {
            println!("Starting Phase {}", i + 1);
            phase = Phase::new(prev_phase, config);
            phase.reset();

            phase.run();

            prev_phase = Some(&phase);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_test() {
        let str = include_str!("utils/input.json");

        let configs: Vec<PhaseConfig> = serde_json::from_str(str).unwrap();

        let sim = Simulation::new(configs);
        sim.run();
    }
}
