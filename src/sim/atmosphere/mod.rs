// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 22.11.23
// Last modified by Tibor Völcker on 28.11.23
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use crate::sim::utils::*;

use self::standard_atmosphere_1962::get_table_row;

mod standard_atmosphere_1962;

pub enum Atmosphere {
    StandardAtmosphere1962,
}

impl Atmosphere {
    fn temperature(&self, alt: f64) -> f64 {
        match self {
            Self::StandardAtmosphere1962 => {
                // T = T_B + L_B * (H_g - H_B)
                let (base_altitude, _, base_temperature, base_temp_gradient) = get_table_row(alt);

                base_temperature + base_temp_gradient * (alt - base_altitude)
            }
        }
    }

    fn pressure(&self, alt: f64) -> f64 {
        match self {
            Self::StandardAtmosphere1962 => {
                // P = P_B * (T_B / T) exp[(g_0*M_0/R*) / L_B] if L_B != 0
                // P = P_B exp[-(g_0*M_0/R*) * (H - H_B) / T_B] if L_B = 0
                let (base_altitude, base_pressure, base_temperature, base_temp_gradient) =
                    get_table_row(alt);
                let temperature = self.temperature(alt);

                if base_temp_gradient != 0. {
                    base_pressure
                        * (base_temperature / temperature)
                        * f64::exp((STD_GRAVITY / AIR_GAS_CONSTANT) / base_temp_gradient)
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

// TODO: Unit tests
