// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 20.12.23
// Last modified by Tibor Völcker on 22.02.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use nalgebra::{vector, Vector3};
use sim::vehicle::{Engine, Steering};
use sim::*;
use std::f64::consts::PI;
use utils::tables::linear_interpolation::Table2D;

#[test]
fn powered_flight() {
    let angular_rate = PI / (10. * 60.);

    let planet = EARTH_SPHERICAL;
    let engine = Engine::new([0., 0.], 1e5, 100., 1.);
    let steering = Steering::Angular(vehicle::Angular::Polynomials(vector![
        0.,
        angular_rate,
        0.,
        0.
    ]));
    let vehicle = Vehicle::new(
        0.,
        Table2D::zeros(),
        Table2D::zeros(),
        Table2D::zeros(),
        vec![engine],
        [None, Some(steering), None],
        f64::INFINITY,
    );
    let r = planet.equatorial_radius;
    let mut sim = Simulation::new(vehicle, planet, 10., [0., 0., PI / 2.]);
    sim.init_inertial(vector![r, 0., 0.], Vector3::zeros());

    let mut state = sim.step();
    while state.time < 10. * 60. {
        println!(
            "Time: {:.0}\nPosition: {:.0}\nVelocity: {:.0}",
            state.time, state.position, state.velocity
        );
        state = sim.step();
    }
}
