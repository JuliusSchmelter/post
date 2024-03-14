// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 22.11.23
// Last modified by Tibor Völcker on 14.03.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use derive_more::{Deref, DerefMut};
use std::f64::consts::PI;

use nalgebra::{vector, Vector3};
use utils::{
    constants::{NEARLY_ZERO, STD_GRAVITY},
    tables::linear_interpolation::Table2D,
};

use crate::planet::ForceState as PlanetState;

fn side_side_angle(a: f64, b: f64, alpha: f64) -> Option<f64> {
    if alpha == 0. {
        if b > a {
            return Some(b - a);
        }
        return Some(a + b);
    }

    if alpha == PI {
        return Some(a - b);
    }

    // No intersection possible (arcsin not defined)
    if alpha.sin() * b > a {
        return None;
    }

    let mut beta = (alpha.sin() * b / a).asin();
    if b > a {
        beta = PI - beta;
    }

    let gamma = PI - beta - alpha;

    Some(a * gamma.sin() / alpha.sin())
}

#[derive(Debug, Default, Clone)]
pub struct Vehicle {
    mass: f64,
    reference_area: f64,
    drag_coeff: Table2D,
    lift_coeff: Table2D,
    side_force_coeff: Table2D,
    engines: Vec<Engine>,
    pub max_acceleration: f64,
}

impl Vehicle {
    pub fn new(
        mass: f64,
        reference_area: f64,
        drag_coeff: Table2D,
        lift_coeff: Table2D,
        side_force_coeff: Table2D,
        engines: Vec<Engine>,
        max_acceleration: f64,
    ) -> Self {
        Self {
            mass,
            reference_area,
            drag_coeff,
            lift_coeff,
            side_force_coeff,
            engines,
            max_acceleration,
        }
    }
}

#[derive(Debug, Default, Deref, DerefMut, Clone)]
pub struct State {
    #[deref]
    #[deref_mut]
    child_state: PlanetState,
    pub acceleration: Vector3<f64>,
    pub propellant_mass: f64,
    pub massflow: f64,
    pub vehicle_acceleration: Vector3<f64>,
    pub throttle: f64,
    pub thrust_force: Vector3<f64>,
    pub aero_force: Vector3<f64>,
    pub alpha: f64,
}

impl Vehicle {
    pub fn force(&self, state: PlanetState) -> State {
        let alpha = self.alpha(
            state
                .inertial_to_body
                .transform_vector(&state.atmos_rel_velocity),
        );
        let aero_force = self.aero_force(alpha, state.mach_number, state.dynamic_pressure);

        let propellant_mass = state.mass - self.mass;

        let throttle = self.auto_throttle(state.mass, state.pressure, aero_force);

        let thrust_force;
        let massflow;
        if propellant_mass <= 0. {
            thrust_force = Vector3::zeros();
            massflow = 0.;
        } else {
            thrust_force = self.thrust_force(throttle, state.pressure);
            massflow = self.massflow(throttle);
        }
        let body_acc = (aero_force + thrust_force) / state.mass;

        // Intersection would require negative thrust
        if body_acc.norm() > self.max_acceleration * 1.001 || throttle.is_nan() {
            panic!("Could not stay in max. acceleration (check aero forces)")
        }

        let acceleration =
            state.body_to_inertial.transform_vector(&body_acc) + state.gravity_acceleration;

        State {
            acceleration,
            propellant_mass,
            massflow,
            vehicle_acceleration: body_acc,
            throttle,
            thrust_force,
            aero_force,
            alpha,
            child_state: state,
        }
    }

    pub fn alpha(&self, velocity: Vector3<f64>) -> f64 {
        if velocity.x < NEARLY_ZERO {
            if velocity.z < NEARLY_ZERO {
                return 0.;
            }
            return velocity.z.signum() * PI / 2.;
        }
        // From [1]: sin(alpha) = z / sqrt(x^2 + z^2)
        //           cos(alpha) = x / sqrt(x^2 + z^2)
        //               alpha = atan(sin(alpha) / cos(alpha))
        // As far as I can see, is the 'sqrt(x^2 + z^2) term useless
        f64::atan(velocity.z / velocity.x)
    }

    pub fn thrust_force(&self, throttle: f64, pressure_atmos: f64) -> Vector3<f64> {
        throttle
            * self
                .engines
                .iter()
                .map(|eng| eng.thrust(pressure_atmos))
                .sum::<Vector3<f64>>()
    }

    pub fn massflow(&self, throttle: f64) -> f64 {
        throttle * self.engines.iter().map(|eng| eng.massflow()).sum::<f64>()
    }

    pub fn auto_throttle(&self, mass: f64, pressure_atmos: f64, aero: Vector3<f64>) -> f64 {
        let max_thrust = self.thrust_force(1., pressure_atmos);

        if max_thrust == Vector3::zeros() {
            // We cannot generate thrust
            return 1.;
        }

        let angle = PI - aero.angle(&max_thrust);

        // Required thrust vector to exactly reach max acceleration
        let opt_req_thrust = side_side_angle(self.max_acceleration * mass, aero.norm(), angle);

        match opt_req_thrust {
            // The clamping can lead to a throttle which violates the maximum acceleration
            // e.g. if the aero forces are very big
            Some(req_thrust) => (req_thrust / max_thrust.norm()).clamp(0., 1.),
            None => 1.,
        }
    }

    pub fn aero_force(&self, alpha: f64, mach: f64, dynamic_pressure: f64) -> Vector3<f64> {
        let cd = self.drag_coeff.at(alpha.to_degrees(), mach);
        let cl = self.lift_coeff.at(alpha.to_degrees(), mach);
        let cy = self.side_force_coeff.at(alpha.to_degrees(), mach);

        let ca = alpha.cos() * cd - alpha.sin() * cl;
        let cn = alpha.sin() * cd + alpha.cos() * cl;

        let mut aero_force = dynamic_pressure * self.reference_area * vector![-ca, cy, -cn];

        // Convert NANs to zeros
        aero_force
            .iter_mut()
            .filter(|i| i.is_nan())
            .for_each(|i| *i = 0.);

        aero_force
    }
}

#[derive(Debug, Clone)]
pub struct Engine {
    // [pitch, yaw]
    incidence: [f64; 2],
    thrust_vac: f64,
    isp_vac: f64,
    exit_area: f64,
}

impl Engine {
    pub fn new(incidence: [f64; 2], thrust_vac: f64, isp_vac: f64, exit_area: f64) -> Self {
        Self {
            incidence,
            thrust_vac,
            isp_vac,
            exit_area,
        }
    }

    fn thrust(&self, pressure_atmos: f64) -> Vector3<f64> {
        vector![
            self.incidence[1].cos() * self.incidence[0].cos(),
            self.incidence[1].sin(),
            self.incidence[1].cos() * self.incidence[0].sin()
        ] * (self.thrust_vac - self.exit_area * pressure_atmos)
    }

    fn massflow(&self) -> f64 {
        -self.thrust_vac / self.isp_vac / STD_GRAVITY
    }
}

#[cfg(test)]
mod tests {
    use crate::example_data::DATA_POINTS;
    use std::ops::Deref;
    use utils::assert_almost_eq_rel;

    #[test]
    fn test_force() {
        const EPSILON: f64 = 0.001;

        for data_point in DATA_POINTS.iter() {
            print!("Testing {} m altitude ... ", data_point.altitude);

            let target = data_point.to_state();
            let input = target.deref();

            let output = data_point.vehicle.force(input.clone());

            assert_almost_eq_rel!(output.throttle, target.throttle, EPSILON);
            assert_almost_eq_rel!(output.massflow, target.massflow, EPSILON);
            assert_almost_eq_rel!(output.propellant_mass, target.propellant_mass, EPSILON);
            assert_almost_eq_rel!(vec output.thrust_force, target.thrust_force, EPSILON);
            assert_almost_eq_rel!(output.alpha.to_degrees(), target.alpha, EPSILON);
            assert_almost_eq_rel!(vec output.aero_force, target.aero_force, EPSILON);
            assert_almost_eq_rel!(vec output.vehicle_acceleration, target.vehicle_acceleration, EPSILON);
            assert_almost_eq_rel!(vec output.acceleration, target.acceleration, EPSILON);

            println!("ok");
        }
    }
}
