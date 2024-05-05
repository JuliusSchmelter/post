// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 04.03.23
// Last modified by Tibor Völcker on 05.05.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

// allow dead code for now, as it's still WIP
#![allow(dead_code)]

mod atmosphere;
mod example_data;
mod integration;
mod phase;
mod planet;
mod state;
mod steering;
mod transformations;
mod utils;
mod vehicle;

pub use phase::Phase;
pub use planet::{Planet, EARTH_FISHER_1960, EARTH_SMITHSONIAN, EARTH_SPHERICAL};
pub use state::State;
pub use steering::Steering;
pub use utils::constants;
pub use vehicle::{Engine, Vehicle};

#[derive(Debug, Default)]
pub struct Simulation {
    phase_inits: Vec<fn(&mut Phase) -> &Phase>,
}

impl Simulation {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_phase(&mut self, init_fn: fn(&mut Phase) -> &Phase) -> &mut Self {
        self.phase_inits.push(init_fn);
        self
    }

    pub fn run(&self) {
        let mut current_phase = Phase::new();
        for (i, phase_init) in self.phase_inits.iter().enumerate() {
            println!("Starting Phase {}", i + 1);
            current_phase.reset();
            phase_init(&mut current_phase);

            current_phase.run()
        }
    }
}

#[cfg(test)]
mod tests {
    use utils::constants::{KILOGRAM_PER_POUND, STD_GRAVITY};

    use self::{
        example_data::{DATA_POINTS, PITCH_RATES, VEHICLES},
        state::StateVariable,
        steering::Axis,
    };

    use super::*;

    #[test]
    fn full_test() {
        let mut sim = Simulation::new();
        // 1. Phase
        sim.add_phase(|phase| {
            phase
                .add_vehicle(VEHICLES[0].clone())
                .add_atmosphere()
                .set_stepsize(5.)
                .init_launch(28.5, 279.4, 90.)
                .set_mass(DATA_POINTS[0].mass)
                .update_termination(StateVariable::Time, 15.)
        })
        // 2. Phase
        .add_phase(|phase| {
            phase
                .update_steering(
                    Axis::Pitch,
                    StateVariable::TimeSinceEvent,
                    [PITCH_RATES[0], 0., 0.],
                )
                .update_termination(StateVariable::Time, 25.)
        })
        // 3. Phase
        .add_phase(|phase| {
            phase
                .update_steering(
                    Axis::Pitch,
                    StateVariable::TimeSinceEvent,
                    [PITCH_RATES[1], 0., 0.],
                )
                .update_termination(StateVariable::Time, 40.)
        })
        // 4. Phase
        .add_phase(|phase| {
            phase
                .update_steering(
                    Axis::Pitch,
                    StateVariable::TimeSinceEvent,
                    [PITCH_RATES[2], 0., 0.],
                )
                .update_termination(StateVariable::Time, 60.)
        })
        // 5. Phase
        .add_phase(|phase| {
            phase
                .update_steering(
                    Axis::Pitch,
                    StateVariable::TimeSinceEvent,
                    [PITCH_RATES[3], 0., 0.],
                )
                .limit_acceleration(3. * STD_GRAVITY)
                .update_termination(StateVariable::Time, 120.)
        })
        // 6. Phase
        .add_phase(|phase| {
            phase
                .update_steering(
                    Axis::Pitch,
                    StateVariable::TimeSinceEvent,
                    [PITCH_RATES[4], 0., 0.],
                )
                .update_termination(StateVariable::Time, 150.)
        })
        // 7. Phase
        .add_phase(|phase| {
            phase
                .update_steering(
                    Axis::Pitch,
                    StateVariable::TimeSinceEvent,
                    [PITCH_RATES[5], 0., 0.],
                )
                .set_stepsize(10.)
                .update_termination(StateVariable::PropellantMass, 0.)
        })
        // 8. Phase
        .add_phase(|phase| {
            phase
                .limit_acceleration(f64::INFINITY)
                .update_termination(StateVariable::TimeSinceEvent, 7.)
        })
        // 9. Phase
        .add_phase(|phase| {
            phase
                .add_vehicle(VEHICLES[1].clone())
                .update_mass(-665000. * KILOGRAM_PER_POUND)
                .update_steering(
                    Axis::Pitch,
                    StateVariable::TimeSinceEvent,
                    [PITCH_RATES[6], 0., 0.],
                )
                .limit_acceleration(3.0 * STD_GRAVITY)
                .set_stepsize(20.)
                .update_termination(StateVariable::TimeSinceEvent, 100.)
        })
        // 10. Phase
        .add_phase(|phase| {
            phase
                .update_steering(
                    Axis::Pitch,
                    StateVariable::TimeSinceEvent,
                    [PITCH_RATES[7], 0., 0.],
                )
                .update_termination(StateVariable::TimeSinceEvent, 150.)
        })
        // 11. Phase
        .add_phase(|phase| phase.update_termination(StateVariable::PropellantMass, 0.));

        sim.run()
    }
}
