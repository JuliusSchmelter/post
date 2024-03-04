// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 06.12.23
// Last modified by Tibor Völcker on 04.03.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::vector;
use sim::*;
use std::f64::consts::PI;
use utils::assert_almost_eq;
use utils::tables::linear_interpolation::Table2D;

#[test]
fn circular_orbit() {
    let planet = EARTH_SPHERICAL;
    let vehicle = Vehicle::new(
        0.,
        Table2D::zeros(),
        Table2D::zeros(),
        Table2D::zeros(),
        vec![],
        f64::INFINITY,
    );
    let r: f64 = 7000e3;
    // v^2 = mu / r
    let v = f64::sqrt(planet.mu() / r);
    // T = 2 PI * sqrt(r^3 / mu)
    let period = 2. * PI * f64::sqrt(r.powi(3) / planet.mu());

    let mut sim = Phase::new(vehicle, planet, 10., move |s| period - s.time);
    sim.init_inertial(vector![r, 0., 0.], vector![0., v, 0.]);

    while !sim.ended {
        sim.step();
        println!(
            "Time: {:.0}\nPosition: {:.0}\nVelocity: {:.0}",
            sim.state.time, sim.state.position, sim.state.velocity
        );

        assert_almost_eq!(sim.state.position.norm(), 7000e3, 10e3);
        assert_eq!(sim.state.position[2], 0.);
        assert_eq!(sim.state.velocity[2], 0.);
    }

    assert_almost_eq!(sim.state.position[0], 7000e3, 10e3);
    assert_almost_eq!(sim.state.position[1].abs(), 0., 50e3);
    assert_almost_eq!(sim.state.velocity[0].abs(), 0., 10e3);
    assert_almost_eq!(sim.state.velocity[1], v, 10.);
}
