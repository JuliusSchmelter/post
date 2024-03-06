// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 04.03.23
// Last modified by Tibor Völcker on 06.03.24
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

pub struct Simulation {}

impl Simulation {
    pub fn new(vehicle: Vehicle, planet: Planet, stepsize: f64) -> Self {
        todo!()
    }

    pub fn add_phase(
        &self,
        init_fn: impl Fn(&mut Phase) -> &Phase,
        termination_fn: impl Fn(&State) -> f64,
    ) {
        todo!()
    }
}

#[cfg(test)]
mod tests {
    use self::{
        example_data::{DATA_POINTS, PITCH_RATES, VEHICLES},
        steering::Axis,
    };

    use super::*;

    #[test]
    fn full_test() {
        let sim = Simulation::new(VEHICLES[0].clone(), EARTH_SPHERICAL, 5.);
        sim.add_phase(
            |phase| {
                phase
                    .add_atmosphere()
                    .init_geodetic(28.5, 279.4, 90.)
                    .init_mass(DATA_POINTS[0].mass)
            },
            |s| 15. - s.time,
        );
        sim.add_phase(
            |phase| phase.update_steering(Axis::Pitch, [0., PITCH_RATES[0], 0.]),
            |s| 25. - s.time,
        );
        sim.add_phase(
            |phase| phase.update_steering(Axis::Pitch, [0., PITCH_RATES[1], 0.]),
            |s| 40. - s.time,
        );
        sim.add_phase(
            |phase| phase.update_steering(Axis::Pitch, [0., PITCH_RATES[2], 0.]),
            |s| 60. - s.time,
        );
        sim.add_phase(
            |phase| {
                phase
                    .update_steering(Axis::Pitch, [0., PITCH_RATES[3], 0.])
                    .limit_acceleration(3.0)
            },
            |s| 120. - s.time,
        );
        sim.add_phase(
            |phase| phase.update_steering(Axis::Pitch, [0., PITCH_RATES[4], 0.]),
            |s| 150. - s.time,
        );
        sim.add_phase(
            |phase| phase.update_steering(Axis::Pitch, [0., PITCH_RATES[5], 0.]),
            |s| s.propellant_mass,
        );
        sim.add_phase(
            |phase| phase.limit_acceleration(f64::INFINITY),
            |s| 7. - s.time_since_event,
        );
        sim.add_phase(
            |phase| {
                phase
                    .update_vehicle(VEHICLES[1].clone())
                    .update_steering(Axis::Pitch, [0., PITCH_RATES[6], 0.])
                    .limit_acceleration(3.0)
            },
            |s| 100. - s.time_since_event,
        );
        sim.add_phase(
            |phase| phase.update_steering(Axis::Pitch, [0., PITCH_RATES[7], 0.]),
            |s| 150. - s.time_since_event,
        );
        sim.add_phase(|phase| phase, |s| s.propellant_mass);
    }
}
