// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 22.11.23
// Last modified by Tibor Völcker on 17.01.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use utils::constants::*;

pub fn temperature(alt: f64) -> f64 {
    // T = T_B + L_B * (H_g - H_B)
    let (base_altitude, _, base_temperature, base_temp_gradient) = get_table_row(alt);

    base_temperature + base_temp_gradient * (alt - base_altitude)
}

pub fn pressure(alt: f64) -> f64 {
    // See [1] p. IV-6
    // P = P_B * (T_B / T)^[(g_0*M_0/R*) / L_B] if L_B != 0
    // P = P_B exp[-(g_0*M_0/R*) * (H - H_B) / T_B] if L_B = 0

    // Watch out: first equation is given as (T_B / T)exp[(g_0*M_0/R*) / L_B],
    // which is supposed to be (T_B / T)^[(g_0*M_0/R*) / L_B]
    let (base_altitude, base_pressure, base_temperature, base_temp_gradient) = get_table_row(alt);
    let temperature = temperature(alt);

    if base_temp_gradient != 0. {
        // Watch out: in this equation in [1], (T_B / T)exp[(g_0*M_0/R*) / L_B] means (T_B / T)^[(g_0*M_0/R*) / L_B]
        // See https://ntrs.nasa.gov/api/citations/19630003300/downloads/19630003300.pdf p. 10
        base_pressure
            * (base_temperature / temperature)
                .powf((STD_GRAVITY / AIR_GAS_CONSTANT) / base_temp_gradient)
    } else {
        base_pressure
            * f64::exp(-(STD_GRAVITY / AIR_GAS_CONSTANT) * (alt - base_altitude) / base_temperature)
    }
}

pub fn density(alt: f64) -> f64 {
    // rho = (M_0/R*) * P / T
    let temperature = temperature(alt);
    let pressure = pressure(alt);
    pressure / (temperature * AIR_GAS_CONSTANT)
}

pub fn speed_of_sound(alt: f64) -> f64 {
    // C_s = (gamma*R*/M_0)^0.5 * T^0.5
    let temperature = temperature(alt);
    f64::sqrt(AIR_KAPPA * AIR_GAS_CONSTANT * temperature)
}

fn get_table_row(geopotational_alt: f64) -> (f64, f64, f64, f64) {
    for i in 1..STD_ATMOS_TABLE.len() {
        if STD_ATMOS_TABLE[i].0 > geopotational_alt {
            return STD_ATMOS_TABLE[i - 1];
        }
    }
    STD_ATMOS_TABLE[STD_ATMOS_TABLE.len()]
}

// TABLE DATA
const STD_ATMOS_TABLE: [(f64, f64, f64, f64); 22] = [
    // [H_B, P_B, T_B, L_B]
    (
        0.0 * METER_PER_FOOT,
        0.21162166e4 * PASCAL_PER_PSF,
        518.67 * KELVIN_PER_RANKIN,
        -0.35661600e-2 * KELVIN_PER_RANKIN / METER_PER_FOOT,
    ),
    (
        36_089.239 * METER_PER_FOOT,
        0.47268050e3 * PASCAL_PER_PSF,
        389.97 * KELVIN_PER_RANKIN,
        0.0 * KELVIN_PER_RANKIN / METER_PER_FOOT,
    ),
    (
        65_616.797 * METER_PER_FOOT,
        0.11434543e3 * PASCAL_PER_PSF,
        389.97 * KELVIN_PER_RANKIN,
        0.54863995e-3 * KELVIN_PER_RANKIN / METER_PER_FOOT,
    ),
    (
        104_986.87 * METER_PER_FOOT,
        0.18128948e2 * PASCAL_PER_PSF,
        411.57 * KELVIN_PER_RANKIN,
        0.15361920e-2 * KELVIN_PER_RANKIN / METER_PER_FOOT,
    ),
    (
        154_199.48 * METER_PER_FOOT,
        0.23163263e1 * PASCAL_PER_PSF,
        487.17 * KELVIN_PER_RANKIN,
        0.0 * KELVIN_PER_RANKIN / METER_PER_FOOT,
    ),
    (
        170_603.68 * METER_PER_FOOT,
        0.12322603e1 * PASCAL_PER_PSF,
        487.17 * KELVIN_PER_RANKIN,
        -0.10972801e-2 * KELVIN_PER_RANKIN / METER_PER_FOOT,
    ),
    (
        200_131.23 * METER_PER_FOOT,
        0.38032532e0 * PASCAL_PER_PSF,
        454.77 * KELVIN_PER_RANKIN,
        -0.21945600e-2 * KELVIN_PER_RANKIN / METER_PER_FOOT,
    ),
    (
        259_186.35 * METER_PER_FOOT,
        0.21673064e-1 * PASCAL_PER_PSF,
        325.17 * KELVIN_PER_RANKIN,
        0.0 * KELVIN_PER_RANKIN / METER_PER_FOOT,
    ),
    (
        291_151.57 * METER_PER_FOOT,
        0.34333824e-2 * PASCAL_PER_PSF,
        325.17 * KELVIN_PER_RANKIN,
        0.16953850e-2 * KELVIN_PER_RANKIN / METER_PER_FOOT,
    ),
    (
        323_002.74 * METER_PER_FOOT,
        0.62814785e-3 * PASCAL_PER_PSF,
        379.17 * KELVIN_PER_RANKIN,
        0.28345707e-2 * KELVIN_PER_RANKIN / METER_PER_FOOT,
    ),
    (
        354_753.59 * METER_PER_FOOT,
        0.15361733e-3 * PASCAL_PER_PSF,
        469.17 * KELVIN_PER_RANKIN,
        0.56867005e-2 * KELVIN_PER_RANKIN / METER_PER_FOOT,
    ),
    (
        396_406.39 * METER_PER_FOOT,
        0.52676024e-4 * PASCAL_PER_PSF,
        649.17 * KELVIN_PER_RANKIN,
        0.11443751e-1 * KELVIN_PER_RANKIN / METER_PER_FOOT,
    ),
    (
        480_781.04 * METER_PER_FOOT,
        0.10566108e-4 * PASCAL_PER_PSF,
        1_729.17 * KELVIN_PER_RANKIN,
        0.86358208e-2 * KELVIN_PER_RANKIN / METER_PER_FOOT,
    ),
    (
        512_046.16 * METER_PER_FOOT,
        0.77263469e-5 * PASCAL_PER_PSF,
        1_999.17 * KELVIN_PER_RANKIN,
        0.57749093e-2 * KELVIN_PER_RANKIN / METER_PER_FOOT,
    ),
    (
        543_215.48 * METER_PER_FOOT,
        0.58405376e-5 * PASCAL_PER_PSF,
        2_179.17 * KELVIN_PER_RANKIN,
        0.40610461e-2 * KELVIN_PER_RANKIN / METER_PER_FOOT,
    ),
    (
        605_268.45 * METER_PER_FOOT,
        0.35246030e-5 * PASCAL_PER_PSF,
        2_431.17 * KELVIN_PER_RANKIN,
        0.29274135e-2 * KELVIN_PER_RANKIN / METER_PER_FOOT,
    ),
    (
        728_243.91 * METER_PER_FOOT,
        0.14559124e-5 * PASCAL_PER_PSF,
        2_791.17 * KELVIN_PER_RANKIN,
        0.23812804e-2 * KELVIN_PER_RANKIN / METER_PER_FOOT,
    ),
    (
        939_894.74 * METER_PER_FOOT,
        0.39418091e-6 * PASCAL_PER_PSF,
        3_295.17 * KELVIN_PER_RANKIN,
        0.20152600e-2 * KELVIN_PER_RANKIN / METER_PER_FOOT,
    ),
    (
        1_234_645.7 * METER_PER_FOOT,
        0.84380249e-7 * PASCAL_PER_PSF,
        3_889.17 * KELVIN_PER_RANKIN,
        0.16354849e-2 * KELVIN_PER_RANKIN / METER_PER_FOOT,
    ),
    (
        1_520_799.4 * METER_PER_FOOT,
        0.22945543e-7 * PASCAL_PER_PSF,
        4_357.17 * KELVIN_PER_RANKIN,
        0.11010085e-2 * KELVIN_PER_RANKIN / METER_PER_FOOT,
    ),
    (
        1_798_726.4 * METER_PER_FOOT,
        0.72259271e-8 * PASCAL_PER_PSF,
        4_663.17 * KELVIN_PER_RANKIN,
        0.73319725e-3 * KELVIN_PER_RANKIN / METER_PER_FOOT,
    ),
    (
        2_068_776.5 * METER_PER_FOOT,
        0.24958752e-8 * PASCAL_PER_PSF,
        4_861.17 * KELVIN_PER_RANKIN,
        0.0 * KELVIN_PER_RANKIN / METER_PER_FOOT,
    ),
];

#[cfg(test)]
mod tests {
    use super::super::Atmosphere;
    use crate::EARTH_SPHERICAL;
    use nalgebra::vector;
    use utils::assert_almost_eq_rel;
    use utils::constants::*;

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
        let mut planet = EARTH_SPHERICAL;
        planet.add_atmosphere(Atmosphere::StandardAtmosphere1962);

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
