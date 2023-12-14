// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 22.11.23
// Last modified by Tibor Völcker on 13.12.23
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use crate::utils::*;

mod standard_atmosphere_1962;

pub enum Atmosphere {
    StandardAtmosphere1962,
}

impl Atmosphere {
    pub fn temperature(&self, alt: f64) -> f64 {
        match self {
            Self::StandardAtmosphere1962 => {
                // T = T_B + L_B * (H_g - H_B)
                let (base_altitude, _, base_temperature, base_temp_gradient) =
                    standard_atmosphere_1962::get_table_row(alt);

                base_temperature + base_temp_gradient * (alt - base_altitude)
            }
        }
    }

    pub fn pressure(&self, alt: f64) -> f64 {
        match self {
            Self::StandardAtmosphere1962 => {
                // See [1] p. IV-6
                // P = P_B * (T_B / T)^[(g_0*M_0/R*) / L_B] if L_B != 0
                // P = P_B exp[-(g_0*M_0/R*) * (H - H_B) / T_B] if L_B = 0

                // Watch out: first equation is given as (T_B / T)exp[(g_0*M_0/R*) / L_B],
                // which is supposed to be (T_B / T)^[(g_0*M_0/R*) / L_B]
                let (base_altitude, base_pressure, base_temperature, base_temp_gradient) =
                    standard_atmosphere_1962::get_table_row(alt);
                let temperature = self.temperature(alt);

                if base_temp_gradient != 0. {
                    // Watch out: in this equation in [1], (T_B / T)exp[(g_0*M_0/R*) / L_B] means (T_B / T)^[(g_0*M_0/R*) / L_B]
                    // See https://ntrs.nasa.gov/api/citations/19630003300/downloads/19630003300.pdf p. 10
                    base_pressure
                        * (base_temperature / temperature)
                            .powf((STD_GRAVITY / AIR_GAS_CONSTANT) / base_temp_gradient)
                } else {
                    base_pressure
                        * f64::exp(
                            -(STD_GRAVITY / AIR_GAS_CONSTANT) * (alt - base_altitude)
                                / base_temperature,
                        )
                }
            }
        }
    }

    pub fn density(&self, alt: f64) -> f64 {
        // rho = (M_0/R*) * P / T
        let temperature = self.temperature(alt);
        let pressure = self.pressure(alt);
        pressure / (temperature * AIR_GAS_CONSTANT)
    }

    pub fn speed_of_sound(&self, alt: f64) -> f64 {
        // C_s = (gamma*R*/M_0)^0.5 * T^0.5
        let temperature = self.temperature(alt);
        f64::sqrt(AIR_KAPPA * AIR_GAS_CONSTANT * temperature)
    }
}

#[cfg(test)]
mod tests {
    mod standard_atmosphere_1962 {
        use super::super::*;
        use crate::{assert_almost_eq_rel, Planet};
        use nalgebra::vector;

        const ATMOSPHERIC_DATA_1967_MODEL: [[f64; 5]; 4] = [
            // [ altitude [m], temperature [K], pressure [Pa], density [kg/m^2], speed of sound [m/s] ]
            // values from https://www.pdas.com/atmoscalculator.html
            [00e3, 288.15, 1.0133e5, 1.22500e-0, 340.294],
            [10e3, 223.25, 2.6500e4, 4.13509e-1, 299.532],
            [20e3, 216.65, 5.5292e3, 8.89083e-2, 295.070],
            [40e3, 250.35, 2.8713e2, 3.99540e-3, 317.190],
            // 1967 atmosphere model differs from 1962 version above 50 km
        ];

        const ATMOSPHERIC_DATA_SS_EXAMPLE: [[f64; 4]; 4] = [
            // values from SS example: [2] p. 277
            [
                -4.76837158e-7 * METER_PER_FOOT,
                5.18670000e2 * KELVIN_PER_RANKIN,
                2.11621660e3 * PASCAL_PER_PSF,
                2.37690697e-3 * KILOGRAM_PER_SLUG / CUBIC_METER_PER_CUBIC_FOOT,
            ],
            [
                9.33310129e2 * METER_PER_FOOT,
                5.15341815e2 * KELVIN_PER_RANKIN,
                2.04581341e3 * PASCAL_PER_PSF,
                2.31267089e-3 * KILOGRAM_PER_SLUG / CUBIC_METER_PER_CUBIC_FOOT,
            ],
            [
                3.04960868e5 * METER_PER_FOOT,
                3.41188380e2 * KELVIN_PER_RANKIN,
                2.01763300e-3 * PASCAL_PER_PSF,
                3.44501515e-9 * KILOGRAM_PER_SLUG / CUBIC_METER_PER_CUBIC_FOOT,
            ],
            [
                3.03804044e5 * METER_PER_FOOT,
                3.39250012e2 * KELVIN_PER_RANKIN,
                2.14880703e-3 * PASCAL_PER_PSF,
                3.68995220e-9 * KILOGRAM_PER_SLUG / CUBIC_METER_PER_CUBIC_FOOT,
            ],
        ];

        fn test_with_data<const N1: usize, const N2: usize>(
            table: [[f64; N1]; N2],
            index: usize,
            epsilon: f64,
        ) {
            let planet = Planet::earth_spherical(Some(Atmosphere::StandardAtmosphere1962), None);

            for data_point in table.iter() {
                print!("Testing {} km altitude ... ", data_point[0]);
                let position = vector![data_point[0], 0., 0.];
                let res = match index {
                    1 => planet.temperature(position),
                    2 => planet.pressure(position),
                    3 => planet.density(position),
                    4 => planet.speed_of_sound(position),
                    _ => panic!(),
                };
                assert_almost_eq_rel!(res, data_point[index], epsilon);
                println!("ok");
            }
        }

        #[test]
        fn test_temperature() {
            let index = 1;
            test_with_data(ATMOSPHERIC_DATA_1967_MODEL, index, 0.002);
            test_with_data(ATMOSPHERIC_DATA_SS_EXAMPLE, index, 0.002);
        }

        #[test]
        fn test_pressure() {
            let index = 2;
            test_with_data(ATMOSPHERIC_DATA_1967_MODEL, index, 0.002);
            test_with_data(ATMOSPHERIC_DATA_SS_EXAMPLE, index, 0.002);
        }

        #[test]
        fn test_density() {
            let index = 3;
            test_with_data(ATMOSPHERIC_DATA_1967_MODEL, index, 0.002);
            test_with_data(ATMOSPHERIC_DATA_SS_EXAMPLE, index, 0.002);
        }

        #[test]
        fn test_speed_of_sound() {
            let index = 4;
            test_with_data(ATMOSPHERIC_DATA_1967_MODEL, index, 0.002);
        }
    }
}
