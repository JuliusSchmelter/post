// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 22.11.23
// Last modified by Tibor Völcker on 13.12.23
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use crate::utils::*;

mod standard_atmosphere_1962;

pub enum Atmosphere {
    StandardAtmosphere1962,
}

impl Atmosphere {
    fn temperature(&self, alt: f64) -> f64 {
        match self {
            Self::StandardAtmosphere1962 => {
                // T = T_B + L_B * (H_g - H_B)
                let (base_altitude, _, base_temperature, base_temp_gradient) =
                    standard_atmosphere_1962::get_table_row(alt);

                base_temperature + base_temp_gradient * (alt - base_altitude)
            }
        }
    }

    fn pressure(&self, alt: f64) -> f64 {
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

    fn density(&self, alt: f64) -> f64 {
        // rho = (M_0/R*) * P / T
        let temperature = self.temperature(alt);
        let pressure = self.pressure(alt);
        pressure / (temperature * AIR_GAS_CONSTANT)
    }

    fn speed_of_sound(&self, alt: f64) -> f64 {
        // C_s = (gamma*R*/M_0)^0.5 * T^0.5
        let temperature = self.temperature(alt);
        f64::sqrt(AIR_KAPPA * AIR_GAS_CONSTANT * temperature)
    }
}

#[cfg(test)]
mod tests {
    const ATMOSPHERIC_DATA: [[f64; 5]; 6] = [
        // [ altitude [m], temperature [K], pressure [Pa], density [kg/m^2], speed of sound [m/s] ]
        // values from https://www.pdas.com/atmoscalculator.html
        [00e3, 288.15, 1.0133e5, 1.22500e-0, 340.294],
        [10e3, 223.25, 2.6500e4, 4.13509e-1, 299.532],
        [20e3, 216.65, 5.5292e3, 8.89083e-2, 295.070],
        [40e3, 250.35, 2.8713e2, 3.99540e-3, 317.190],
        [60e3, 247.02, 2.1955e1, 3.09636e-4, 315.072],
        [80e3, 198.63, 1.0521e0, 1.84524e-5, 282.535],
    ];

    mod standard_atmosphere_1962 {
        use super::super::*;
        use super::*;
        use crate::{assert_almost_eq_rel, Planet};

        #[test]
        fn test_temperature() {
            let atmos = Atmosphere::StandardAtmosphere1962;
            let planet = Planet::earth_fisher_1960(None);

            for data_point in ATMOSPHERIC_DATA.iter() {
                let geopotential_alt = planet.geopotational_altitude(data_point[0]);
                let mut epsilon = 0.005;
                if data_point[0] > 50e3 {
                    // Data points are from 1967 atmosphere model, which is a bit different from 1962 version above 50 km
                    epsilon = 0.1;
                }
                print!("Testing {} km altitude ... ", data_point[0]);
                assert_almost_eq_rel!(atmos.temperature(geopotential_alt), data_point[1], epsilon);
                println!("ok");
            }
        }

        #[test]
        fn test_pressure() {
            let atmos = Atmosphere::StandardAtmosphere1962;
            let planet = Planet::earth_fisher_1960(None);

            for data_point in ATMOSPHERIC_DATA.iter() {
                let geopotential_alt = planet.geopotational_altitude(data_point[0]);
                let mut epsilon = 0.005;
                if data_point[0] > 50e3 {
                    // Data points are from 1967 atmosphere model, which is a bit different from 1962 version above 50 km
                    epsilon = 0.03;
                }
                print!("Testing {} km altitude ... ", data_point[0]);
                assert_almost_eq_rel!(atmos.pressure(geopotential_alt), data_point[2], epsilon);
                println!("ok");
            }
        }

        #[test]
        fn test_density() {
            let atmos = Atmosphere::StandardAtmosphere1962;
            let planet = Planet::earth_fisher_1960(None);

            for data_point in ATMOSPHERIC_DATA.iter() {
                let geopotential_alt = planet.geopotational_altitude(data_point[0]);
                let mut epsilon = 0.005;
                if data_point[0] > 50e3 {
                    // Data points are from 1967 atmosphere model, which is a bit different from 1962 version above 50 km
                    epsilon = 0.1;
                }
                print!("Testing {} km altitude ... ", data_point[0]);
                assert_almost_eq_rel!(atmos.density(geopotential_alt), data_point[3], epsilon);
                println!("ok");
            }
        }

        #[test]
        fn test_speed_of_sound() {
            let atmos = Atmosphere::StandardAtmosphere1962;
            let planet = Planet::earth_fisher_1960(None);

            for data_point in ATMOSPHERIC_DATA.iter() {
                let geopotential_alt = planet.geopotational_altitude(data_point[0]);
                let mut epsilon = 0.005;
                if data_point[0] > 50e3 {
                    // Data points are from 1967 atmosphere model, which is a bit different from 1962 version above 50 km
                    epsilon = 0.05;
                }
                print!("Testing {} km altitude ... ", data_point[0]);
                assert_almost_eq_rel!(
                    atmos.speed_of_sound(geopotential_alt),
                    data_point[4],
                    epsilon
                );
                println!("ok");
            }
        }
    }
}
