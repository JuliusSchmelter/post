// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 06.12.23
// Last modified by Tibor Völcker on 06.12.23
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::vector;
use sim::integration::RK4;
use sim::vehicle::{Angular, Steering};
use sim::*;
use std::f64::consts::PI;

#[test]
fn circular_orbit() {
    let planet = Planet::earth_spherical(None);
    let vehicle = Vehicle::new(
        10e3,
        vec![],
        Steering::Angular(Angular::Polynomials(vector![0., 0., 0., 0.])),
    );
    let r: f64 = 7000e3;
    // v^2 = mu / r
    let v = f64::sqrt(planet.mu() / r);
    // T = 2 PI * sqrt(r^3 / mu)
    let period = 2. * PI * f64::sqrt(r.powi(3) / planet.mu());

    let mut system = TranslationalEquations::new(vehicle, planet);
    system.vehicle.position = vector![r, 0., 0.];
    system.vehicle.velocity = vector![0., v, 0.];

    while system.time < period {
        RK4.step(&mut system, 10.);
        println!(
            "Time: {:.0}\nPosition: {:.0}\nVelocity: {:.0}",
            system.time, system.vehicle.position, system.vehicle.velocity
        );
        assert_almost_eq!(system.vehicle.position.norm(), 7000e3, 10e3);
        assert_eq!(system.vehicle.position[2], 0.);
        assert_eq!(system.vehicle.velocity[2], 0.);
    }

    assert_almost_eq!(system.vehicle.position[0], 7000e3, 10e3);
    assert_almost_eq!(system.vehicle.position[1].abs(), 0., 50e3);
    assert_almost_eq!(system.vehicle.velocity[0].abs(), 0., 10e3);
    assert_almost_eq!(system.vehicle.velocity[1], v, 10.);
}
