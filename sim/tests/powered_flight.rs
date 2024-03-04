// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 20.12.23
// Last modified by Tibor Völcker on 04.03.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use sim::Engine;
use sim::*;
use std::f64::consts::PI;
use utils::tables::linear_interpolation::Table2D;

#[test]
fn powered_flight() {
    let angular_rate = PI / (10. * 60.);

    let planet = EARTH_SPHERICAL;
    let engine = Engine::new([0., 0.], 1e5, 100., 1.);
    let vehicle = Vehicle::new(
        0.,
        Table2D::zeros(),
        Table2D::zeros(),
        Table2D::zeros(),
        vec![engine],
        f64::INFINITY,
    );
    let mut sim = Phase::new(vehicle, planet, 10., |s| 10. * 60. - s.time);
    sim.add_steering(1, [0., angular_rate, 0., 0.]);
    sim.init_geodetic(0., 0., 90.);

    while !sim.ended {
        sim.step();
        println!(
            "Time: {:.0}\nPosition: {:.0}\nVelocity: {:.0}",
            sim.state.time, sim.state.position, sim.state.velocity
        );
    }
}
