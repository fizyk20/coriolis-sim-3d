use std::f64::consts::PI;

use nalgebra::Vector3;

use crate::simulation::{Object, Position, Velocity};

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
    attractor_coeff: f64,
    vel: f64,
    vel_up: f64,
    num_objects: usize,
    color: (f32, f32, f32),
) -> Vec<Object> {
    let center_pos = Position::from_lat_lon_elev(lat, lon, elev);
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
                .with_attractor(Box::new(move |pos| {
                    let pos_diff = center_pos.to_omega(pos.omega()).pos() - pos.pos();
                    let pos_norm = pos_diff.norm();
                    pos_diff / pos_norm / pos_norm * attractor_coeff
                }))
        })
        .collect()
}
