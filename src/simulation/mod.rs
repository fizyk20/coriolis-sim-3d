mod object;

use std::f64::consts::PI;

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

pub fn earth_radius(lat_r_gc: f64) -> f64 {
    let x = lat_r_gc.cos() / R_EQU;
    let y = lat_r_gc.sin() / R_POL;
    1.0 / (x * x + y * y).sqrt()
}

pub fn surface_normal(pos: &Vector3<f64>) -> Vector3<f64> {
    let v = Vector3::new(
        pos.x / R_EQU / R_EQU,
        pos.y / R_POL / R_POL,
        pos.z / R_EQU / R_EQU,
    );
    let v_norm = v.norm();
    v / v_norm
}

pub fn r_curv(pos: &Vector3<f64>) -> f64 {
    let r2 = pos.dot(&pos);
    let coeff = (R_EQU * R_EQU + R_POL * R_POL - r2).sqrt();
    coeff * coeff * coeff / R_EQU / R_POL
}

fn get_coords_at_dist(lat: f64, lon: f64, dir: f64, dist: f64) -> (f64, f64) {
    let lat = lat.to_radians();
    let lon = lon.to_radians();
    let dir = dir.to_radians();
    let ang = dist / 6371e3;

    let v_pos = Vector3::new(lat.cos() * lon.cos(), lat.cos() * lon.sin(), lat.sin());
    let v_e = Vector3::new(-lon.sin(), lon.cos(), 0.0);
    let v_n = v_pos.cross(&v_e);
    let v_dir = v_n * dir.cos() + v_e * dir.sin();

    let fpos = v_pos * ang.cos() + v_dir * ang.sin();
    let fpos = fpos / fpos.norm();
    let new_lat = fpos.z.asin().to_degrees();
    let new_lon = fpos.y.atan2(fpos.x).to_degrees();

    (new_lat, new_lon)
}

pub fn anticyclone(
    lat: f64,
    lon: f64,
    elev: f64,
    vel: f64,
    vel_up: f64,
    num_objects: usize,
    color: (f32, f32, f32),
) -> Vec<Object> {
    let pos = Position::from_lat_lon_elev(lat, lon, elev);
    (0..num_objects)
        .into_iter()
        .map(|index| {
            let azim = 2.0 * PI / (num_objects as f64) * (index as f64);
            let vel_n = vel * azim.cos();
            let vel_e = vel * azim.sin();
            let vel = Velocity::from_east_north_up(pos, vel_e, vel_n, vel_up);
            Object::new(pos, vel)
                .with_color(color.0, color.1, color.2)
                .with_radius(100e3)
        })
        .collect()
}

pub fn cyclone(
    lat: f64,
    lon: f64,
    elev: f64,
    radius: f64,
    vel: f64,
    vel_up: f64,
    num_objects: usize,
    color: (f32, f32, f32),
) -> Vec<Object> {
    (0..num_objects)
        .into_iter()
        .map(|index| {
            let azim = 2.0 * PI / (num_objects as f64) * (index as f64);
            let (nlat, nlon) = get_coords_at_dist(lat, lon, azim.to_degrees(), radius);
            let pos = Position::from_lat_lon_elev(nlat, nlon, elev);
            let vel_n = -vel * azim.cos();
            let vel_e = -vel * azim.sin();
            let vel = Velocity::from_east_north_up(pos, vel_e, vel_n, vel_up);
            Object::new(pos, vel)
                .with_color(color.0, color.1, color.2)
                .with_radius(100e3)
        })
        .collect()
}
