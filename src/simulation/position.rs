use nalgebra::Vector3;

use super::{lat_lon_elev_to_vec3, OMEGA};

#[derive(Debug, Clone, Copy)]
pub struct Position {
    t: f64,
    pos: Vector3<f64>,
    // angular velocity of the frame of reference
    omega: f64,
}

impl Position {
    pub fn from_lat_lon_elev(lat: f64, lon: f64, elev: f64) -> Self {
        let pos = lat_lon_elev_to_vec3(lat, lon, elev);
        Self {
            t: 0.0,
            pos,
            omega: OMEGA,
        }
    }

    pub fn with_t(self, t: f64) -> Self {
        Position { t, ..self }
    }

    pub fn to_omega(self, omega: f64) -> Self {
        if self.omega == omega {
            return self;
        }
        let Position {
            t,
            pos,
            omega: omega_old,
        } = self;
        let wt = (omega_old - omega) * t;
        let s = wt.sin();
        let c = wt.cos();
        let pos = Vector3::new(pos.x * c + pos.z * s, pos.y, -pos.x * s + pos.z * c);
        Position { t, pos, omega }
    }

    pub fn dir_to_omega(&self, vec: Vector3<f64>, omega: f64) -> Vector3<f64> {
        let w = omega - self.omega;
        let wt = w * self.t;
        let s = wt.sin();
        let c = wt.cos();

        Vector3::new(vec.x * c - vec.z * s, vec.y, vec.x * s + vec.z * c)
    }

    pub fn grav(&self, gm: f64) -> Vector3<f64> {
        let r = self.pos.norm();
        -gm / r / r / r * self.pos
    }

    pub fn centrifugal(&self) -> Vector3<f64> {
        let r_xz = Vector3::new(self.pos.x, 0.0, self.pos.z);
        r_xz * self.omega * self.omega
    }

    pub fn t(&self) -> f64 {
        self.t
    }

    pub fn omega(&self) -> f64 {
        self.omega
    }

    pub fn pos(&self) -> Vector3<f64> {
        self.pos
    }

    pub fn increase(&mut self, v: Vector3<f64>) {
        self.pos += v;
    }

    pub fn increase_time(&mut self, dt: f64) {
        self.t += dt;
    }

    pub fn mul(&mut self, x: f64) {
        self.pos *= x;
    }
}
