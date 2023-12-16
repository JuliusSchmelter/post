// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 06.12.23
// Last modified by Tibor Völcker on 16.12.23
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::vector;
use sim::*;
use std::f64::consts::PI;

#[test]
fn circular_orbit() {
    let planet = EARTH_SPHERICAL;
    let vehicle = Vehicle::new(10e3, vec![], None);
    let r: f64 = 7000e3;
    // v^2 = mu / r
    let v = f64::sqrt(planet.mu() / r);
    // T = 2 PI * sqrt(r^3 / mu)
    let period = 2. * PI * f64::sqrt(r.powi(3) / planet.mu());

    let mut sim = Simulation::new(vehicle, planet, 10.);
    sim.vehicle.position = vector![r, 0., 0.];
    sim.vehicle.velocity = vector![0., v, 0.];

    while sim.time < period {
        sim.step();
        println!(
            "Time: {:.0}\nPosition: {:.0}\nVelocity: {:.0}",
            sim.time, sim.vehicle.position, sim.vehicle.velocity
        );
        assert_almost_eq!(sim.vehicle.position.norm(), 7000e3, 10e3);
        assert_eq!(sim.vehicle.position[2], 0.);
        assert_eq!(sim.vehicle.velocity[2], 0.);
    }

    assert_almost_eq!(sim.vehicle.position[0], 7000e3, 10e3);
    assert_almost_eq!(sim.vehicle.position[1].abs(), 0., 50e3);
    assert_almost_eq!(sim.vehicle.velocity[0].abs(), 0., 10e3);
    assert_almost_eq!(sim.vehicle.velocity[1], v, 10.);
}
