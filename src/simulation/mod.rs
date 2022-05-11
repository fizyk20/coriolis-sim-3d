mod object;

use nalgebra::Vector3;

pub use object::{Object, Position, Velocity};

/// Earth's angular speed in radians per second
pub const OMEGA: f64 = 7.29212351699e-5;
/// Earth's mass multiplied by G in m³/s²
pub const GM: f64 = 3.986004418e14;

/// Earth's equatorial radius
pub const R_EQU: f64 = 6_378_137.0;
/// Earth's polar radius
pub const R_POL: f64 = 6_356_752.0;
/// Earth's oblateness
pub const ECC2: f64 = (R_EQU * R_EQU - R_POL * R_POL) / R_EQU / R_EQU;

fn nphi(lat_r: f64) -> f64 {
    R_EQU / (1.0 - ECC2 * lat_r.sin() * lat_r.sin()).sqrt()
}

fn pr(lat_r: f64, elev: f64) -> f64 {
    (nphi(lat_r) + elev) * lat_r.cos()
}

fn pz(lat_r: f64, elev: f64) -> f64 {
    (nphi(lat_r) * (1.0 - ECC2) + elev) * lat_r.sin()
}

pub fn lat_lon_elev_to_vec3(lat: f64, lon: f64, elev: f64) -> Vector3<f64> {
    let lat = lat.to_radians();
    let lon = lon.to_radians();

    let x = pr(lat, elev);
    let y = pz(lat, elev);

    Vector3::new(x * lon.sin(), y, x * lon.cos())
}
