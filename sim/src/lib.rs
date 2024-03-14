// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 04.03.23
// Last modified by Tibor Völcker on 14.03.24
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
mod vehicle;

pub use phase::Phase;
pub use planet::{Planet, EARTH_FISHER_1960, EARTH_SMITHSONIAN, EARTH_SPHERICAL};
pub use state::State;
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
    use utils::constants::STD_GRAVITY;

    use self::{
        example_data::{DATA_POINTS, PITCH_RATES, VEHICLES},
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
                .update_termination(|s| 15. - s.time)
        })
        // 2. Phase
        .add_phase(|phase| {
            phase
                .update_steering(Axis::Pitch, [PITCH_RATES[0], 0., 0.])
                .update_termination(|s| 25. - s.time)
        })
        // 3. Phase
        .add_phase(|phase| {
            phase
                .update_steering(Axis::Pitch, [PITCH_RATES[1], 0., 0.])
                .update_termination(|s| 40. - s.time)
        })
        // 4. Phase
        .add_phase(|phase| {
            phase
                .update_steering(Axis::Pitch, [PITCH_RATES[2], 0., 0.])
                .update_termination(|s| 60. - s.time)
        })
        // 5. Phase
        .add_phase(|phase| {
            phase
                .update_steering(Axis::Pitch, [PITCH_RATES[3], 0., 0.])
                .limit_acceleration(3. * STD_GRAVITY)
                .update_termination(|s| 120. - s.time)
        })
        // 6. Phase
        .add_phase(|phase| {
            phase
                .update_steering(Axis::Pitch, [PITCH_RATES[4], 0., 0.])
                .update_termination(|s| 150. - s.time)
        })
        // 7. Phase
        .add_phase(|phase| {
            phase
                .update_steering(Axis::Pitch, [PITCH_RATES[5], 0., 0.])
                .set_stepsize(10.)
                .update_termination(|s| s.propellant_mass)
        })
        // 8. Phase
        .add_phase(|phase| {
            phase
                .limit_acceleration(f64::INFINITY)
                .update_termination(|s| 7. - s.time_since_event)
        })
        // 9. Phase
        .add_phase(|phase| {
            phase
                .add_vehicle(VEHICLES[1].clone())
                .update_steering(Axis::Pitch, [PITCH_RATES[6], 0., 0.])
                .limit_acceleration(3.0)
                .set_stepsize(20.)
                .update_termination(|s| 100. - s.time_since_event)
        })
        // 10. Phase
        .add_phase(|phase| {
            phase
                .update_steering(Axis::Pitch, [PITCH_RATES[7], 0., 0.])
                .update_termination(|s| 150. - s.time_since_event)
        })
        // 11. Phase
        .add_phase(|phase| phase.update_termination(|s| s.propellant_mass));

        sim.run()
    }
}
