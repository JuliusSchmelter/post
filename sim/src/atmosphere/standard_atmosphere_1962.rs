// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 22.11.23
// Last modified by Tibor Völcker on 05.03.24
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
    STD_ATMOS_TABLE[STD_ATMOS_TABLE.len() - 1]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::example_data::DATA_POINTS;
    use crate::EARTH_SPHERICAL;
    use utils::assert_almost_eq_rel;

    #[test]
    fn test_example_data() {
        const EPSILON: f64 = 0.001;

        for data_point in DATA_POINTS.iter() {
            print!("Testing {} km altitude ... ", data_point.altitude);
            let alt = EARTH_SPHERICAL.geopotational_altitude(data_point.position);
            assert_almost_eq_rel!(temperature(alt), data_point.temperature, EPSILON);
            assert_almost_eq_rel!(pressure(alt), data_point.pressure, EPSILON);
            assert_almost_eq_rel!(density(alt), data_point.density, EPSILON);
            println!("ok");
        }
    }
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
