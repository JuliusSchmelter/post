// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 06.12.23
// Last modified by Tibor Völcker on 19.01.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use sim::*;
use std::f64::consts::PI;
use utils::assert_almost_eq;
use utils::tables::linear_interpolation::Table2D;

#[test]
fn circular_orbit() {
    let planet = EARTH_SPHERICAL;
    let vehicle = Vehicle::new(
        10e3,
        0.,
        Table2D::zeros(),
        Table2D::zeros(),
        Table2D::zeros(),
        vec![],
        [None, None, None],
        f64::INFINITY,
    );
    let r: f64 = 7000e3;
    // v^2 = mu / r
    let v = f64::sqrt(planet.mu() / r);
    // T = 2 PI * sqrt(r^3 / mu)
    let period = 2. * PI * f64::sqrt(r.powi(3) / planet.mu());

    let mut sim = Simulation::new(vehicle, planet, 10., [0., 0., 0.]);
    sim.set_position(&[r, 0., 0.]);
    sim.set_velocity(&[0., v, 0.]);

    while sim.time < period {
        sim.step();
        println!(
            "Time: {:.0}\nPosition: {:.0}\nVelocity: {:.0}",
            sim.time,
            sim.position(),
            sim.velocity()
        );
        assert_almost_eq!(sim.position().norm(), 7000e3, 10e3);
        assert_eq!(sim.position()[2], 0.);
        assert_eq!(sim.velocity()[2], 0.);
    }

    assert_almost_eq!(sim.position()[0], 7000e3, 10e3);
    assert_almost_eq!(sim.position()[1].abs(), 0., 50e3);
    assert_almost_eq!(sim.velocity()[0].abs(), 0., 10e3);
    assert_almost_eq!(sim.velocity()[1], v, 10.);
}
