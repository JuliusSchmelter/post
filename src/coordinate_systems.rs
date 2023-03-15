pub enum Length {
    Meter(f64),
    Kilometer(f64),
}
pub enum Angle {
    Degree(f64),
}

// Earth-centered inertial (ECI) axes
pub struct ECI {
    x: Length,
    y: Length,
    z: Length,
}

// Earth-centered rotating (ECR) axes
pub struct ECR {
    x: Length,
    y: Length,
    z: Length,
}

// Earth position coordinates
pub struct Geodetic {
    latitude: Angle,
    longitude: Angle,
    altitude: Length,
}

// Geographic (G) axes
pub struct LocalLevel {
    x: Length,
    y: Length,
    z: Length,
}

// Inertial launch (L) axes
pub struct Launch {
    x: Length,
    y: Length,
    z: Length,
}

// Body (B) axes
pub struct Body {
    x: Length,
    y: Length,
    z: Length,
}

// Body reference (BR) axes
pub struct BodyReference {
    x: Length,
    y: Length,
    z: Length,
}

// Orbital elements
pub struct OrbitalElements {
    apogee_altitude: Length,
    perigee_altitude: Length,
    inclination: Angle,
    longitude_ascending_node: Angle,
    true_anomaly: Angle,
    argument_perigee: Angle,
}

// Vernal Equinox (VE) Axes
pub struct VernalEquinox {
    x: Length,
    y: Length,
    z: Length,
}
