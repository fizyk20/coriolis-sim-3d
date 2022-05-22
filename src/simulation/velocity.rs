use nalgebra::Vector3;

use super::{Position, GM, OMEGA};

#[derive(Debug, Clone, Copy)]
pub struct Velocity {
    vel: Vector3<f64>,
    // angular velocity of the frame of reference
    omega: f64,
}

impl Velocity {
    pub fn from_east_north_up(pos: Position, e: f64, n: f64, u: f64) -> Self {
        let old_omega = pos.omega();
        let pos = pos.to_omega(OMEGA);
        let eff_grav = pos.grav(GM) + pos.centrifugal();
        let up = -eff_grav / eff_grav.norm();
        let lon = pos.pos().x.atan2(pos.pos().z);
        let east = Vector3::new(lon.cos(), 0.0, -lon.sin());
        let north = up.cross(&east);

        let vel = Self {
            vel: e * east + n * north + u * up,
            omega: OMEGA,
        };

        vel.to_omega(pos, old_omega)
    }

    pub fn to_omega(self, pos: Position, omega: f64) -> Self {
        if self.omega == omega {
            return self;
        }
        let Velocity {
            vel,
            omega: omega_old,
        } = self;

        let pos = pos.to_omega(omega_old);
        let t = pos.t();
        let pos = pos.pos();

        let dw = omega - omega_old;
        let wt = dw * t;
        let s = wt.sin();
        let c = wt.cos();

        let z2 = vel.z + pos.x * dw;
        let x2 = vel.x - pos.z * dw;
        let vel = Vector3::new(x2 * c - z2 * s, vel.y, x2 * s + z2 * c);

        Velocity { vel, omega }
    }

    pub fn coriolis(&self) -> Vector3<f64> {
        let omega_v = Vector3::new(0.0, self.omega, 0.0);
        -2.0 * omega_v.cross(&self.vel)
    }

    pub fn vel(&self) -> Vector3<f64> {
        self.vel
    }

    pub fn omega(&self) -> f64 {
        self.omega
    }

    pub fn increase(&mut self, v: Vector3<f64>) {
        self.vel += v;
    }

    pub fn mul(&mut self, x: f64) {
        self.vel *= x;
    }
}
