use std::fmt;

use crate::simulation::{Object, Position, Velocity};

use super::utils::*;

#[derive(Clone, PartialEq)]
pub enum ObjectKindTag {
    Free,
    Cyclone,
    Anticyclone,
    Foucault,
}

impl fmt::Display for ObjectKindTag {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            ObjectKindTag::Free => write!(f, "Free"),
            ObjectKindTag::Cyclone => write!(f, "Cyclone"),
            ObjectKindTag::Anticyclone => write!(f, "Anticyclone"),
            ObjectKindTag::Foucault => write!(f, "Foucault Pendulum"),
        }
    }
}

#[derive(Clone)]
pub enum ObjectKind {
    Free {
        vel_n: String,
        vel_e: String,
        vel_u: String,
    },
    Cyclone {
        n_particles: String,
        radius: String,
        vel: String,
    },
    Anticyclone {
        n_particles: String,
        vel: String,
    },
    Foucault {
        vel: String,
        azim: String,
    },
}

impl ObjectKind {
    pub fn default_free() -> Self {
        Self::Free {
            vel_n: "0".to_string(),
            vel_e: "0".to_string(),
            vel_u: "0".to_string(),
        }
    }

    pub fn default_cyclone() -> Self {
        Self::Cyclone {
            n_particles: "8".to_string(),
            radius: "1000".to_string(),
            vel: "100".to_string(),
        }
    }

    pub fn default_anticyclone() -> Self {
        Self::Anticyclone {
            n_particles: "8".to_string(),
            vel: "100".to_string(),
        }
    }

    pub fn default_foucault() -> Self {
        Self::Foucault {
            vel: "2000".to_string(),
            azim: "0".to_string(),
        }
    }

    pub fn as_tag(&self) -> ObjectKindTag {
        match self {
            ObjectKind::Free { .. } => ObjectKindTag::Free,
            ObjectKind::Cyclone { .. } => ObjectKindTag::Cyclone,
            ObjectKind::Anticyclone { .. } => ObjectKindTag::Anticyclone,
            ObjectKind::Foucault { .. } => ObjectKindTag::Foucault,
        }
    }
}

#[derive(Clone)]
pub struct ObjectDescription {
    pub lat: String,
    pub lon: String,
    pub elev: String,
    pub color: [f32; 3],
    pub kind: ObjectKind,
}

impl Default for ObjectDescription {
    fn default() -> Self {
        Self {
            lat: "0".to_string(),
            lon: "0".to_string(),
            elev: "0".to_string(),
            color: [1.0, 0.0, 0.0],
            kind: ObjectKind::default_free(),
        }
    }
}

impl ObjectDescription {
    fn lat_f(&self) -> f64 {
        self.lat.parse().unwrap_or(0.0)
    }

    fn lon_f(&self) -> f64 {
        self.lon.parse().unwrap_or(0.0)
    }

    fn elev_f(&self) -> f64 {
        self.elev.parse().unwrap_or(0.0)
    }

    pub fn into_objects(&self) -> Vec<Object> {
        match &self.kind {
            ObjectKind::Free {
                vel_n,
                vel_e,
                vel_u,
            } => {
                let vel_e = vel_e.parse().unwrap_or(0.0);
                let vel_n = vel_n.parse().unwrap_or(0.0);
                let vel_u = vel_u.parse().unwrap_or(0.0);
                vec![create_object(
                    self.lat_f(),
                    self.lon_f(),
                    self.elev_f(),
                    vel_e,
                    vel_n,
                    vel_u,
                )
                .with_color(self.color[0], self.color[1], self.color[2])]
            }
            ObjectKind::Cyclone {
                n_particles,
                radius,
                vel,
            } => {
                let n_particles = n_particles.parse().unwrap_or(0);
                let radius = radius.parse().unwrap_or(0.0);
                let vel = vel.parse().unwrap_or(0.0);
                cyclone(
                    self.lat_f(),
                    self.lon_f(),
                    self.elev_f(),
                    radius * 1000.0_f64,
                    2e4,
                    vel,
                    0.0,
                    n_particles,
                    (self.color[0], self.color[1], self.color[2]),
                )
            }
            ObjectKind::Anticyclone { n_particles, vel } => {
                let n_particles = n_particles.parse().unwrap_or(0);
                let vel = vel.parse().unwrap_or(0.0);
                anticyclone(
                    self.lat_f(),
                    self.lon_f(),
                    self.elev_f(),
                    vel,
                    0.0,
                    n_particles,
                    (self.color[0], self.color[1], self.color[2]),
                )
            }
            ObjectKind::Foucault { vel, azim } => {
                let azim = azim.parse().unwrap_or(0.0f64).to_radians();
                let vel = vel.parse().unwrap_or(0.0);
                let vel_e = vel * azim.sin();
                let vel_n = vel * azim.cos();
                vec![
                    create_object(self.lat_f(), self.lon_f(), self.elev_f(), vel_e, vel_n, 0.0)
                        .with_color(self.color[0], self.color[1], self.color[2])
                        .as_pendulum(2e-6),
                ]
            }
        }
    }
}

#[derive(Clone)]
pub struct InitialStateDefinition {
    pub selected_kind: ObjectKindTag,
    pub objects: Vec<ObjectDescription>,
}

impl Default for InitialStateDefinition {
    fn default() -> Self {
        Self {
            selected_kind: ObjectKindTag::Free,
            objects: vec![],
        }
    }
}

fn create_object(lat: f64, lon: f64, elev: f64, v_e: f64, v_n: f64, v_u: f64) -> Object {
    let pos = Position::from_lat_lon_elev(lat, lon, elev);
    let vel = Velocity::from_east_north_up(pos, v_e, v_n, v_u);
    Object::new(pos, vel)
}
