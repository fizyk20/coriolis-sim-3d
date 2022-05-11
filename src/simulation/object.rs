use std::collections::VecDeque;

use glium::{index, uniform, Display, DrawParameters, Frame, Program, Surface, VertexBuffer};
use nalgebra::{base::dimension::U7, Matrix4, Vector3, VectorN};
use numeric_algs::{
    integration::{Integrator, StepSize},
    State,
};

use super::{earth_radius, lat_lon_elev_to_vec3, r_curv, surface_normal, GM, OMEGA};
use crate::renderer::{Mesh, Vertex};

#[derive(Debug, Clone, Copy)]
pub struct Position {
    pub t: f64,
    pub pos: Vector3<f64>,
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

    fn grav(&self, gm: f64) -> Vector3<f64> {
        let r = self.pos.norm();
        -gm / r / r / r * self.pos
    }

    fn centrifugal(&self) -> Vector3<f64> {
        let r_xz = Vector3::new(self.pos.x, 0.0, self.pos.z);
        r_xz * self.omega * self.omega
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Velocity {
    pub vel: Vector3<f64>,
    // angular velocity of the frame of reference
    omega: f64,
}

impl Velocity {
    pub fn from_east_north_up(pos: Position, e: f64, n: f64, u: f64) -> Self {
        let old_omega = pos.omega;
        let pos = pos.to_omega(OMEGA);
        let eff_grav = pos.grav(GM) + pos.centrifugal();
        let up = -eff_grav / eff_grav.norm();
        let lon = pos.pos.x.atan2(pos.pos.z);
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
        let Position { t, pos, .. } = pos.to_omega(omega_old);
        let dw = omega - omega_old;
        let wt = dw * t;
        let s = wt.sin();
        let c = wt.cos();

        let z2 = vel.z + pos.x * dw;
        let x2 = vel.x - pos.z * dw;
        let vel = Vector3::new(x2 * c - z2 * s, vel.y, x2 * s + z2 * c);

        Velocity { vel, omega }
    }
}

const MAX_PATH_LEN: usize = 5000;

#[derive(Debug, Clone, Copy)]
enum ObjectState {
    InFlight,
    OnSurface,
}

#[derive(Debug, Clone)]
pub struct Object {
    pub pos: Position,
    pub vel: Velocity,
    color: (f32, f32, f32),
    path: VecDeque<Position>,
    gm: f64,
    drag_coeff: f64,
    state: ObjectState,
}

impl Object {
    pub fn new(pos: Position, vel: Velocity) -> Self {
        Self {
            pos,
            vel,
            color: (1.0, 0.0, 0.0),
            path: VecDeque::new(),
            gm: GM,
            drag_coeff: 0.0,
            state: ObjectState::InFlight,
        }
    }

    pub fn with_color(self, r: f32, g: f32, b: f32) -> Self {
        Self {
            color: (r, g, b),
            ..self
        }
    }

    pub fn with_gm(self, gm: f64) -> Self {
        Self { gm, ..self }
    }

    pub fn with_drag(self, drag: f64) -> Self {
        Self {
            drag_coeff: drag,
            ..self
        }
    }

    pub fn time(&self) -> f64 {
        self.pos.t
    }

    fn coriolis(&self) -> Vector3<f64> {
        let omega_v = Vector3::new(0.0, self.pos.omega, 0.0);
        let vel = self.vel.to_omega(self.pos, self.pos.omega);
        -2.0 * omega_v.cross(&vel.vel)
    }

    fn derivative_inflight(&self) -> VectorN<f64, U7> {
        let vel = self.vel.to_omega(self.pos, self.pos.omega).vel;
        let acc = self.pos.grav(self.gm) + self.pos.centrifugal() + self.coriolis();

        VectorN::<f64, U7>::from_column_slice(&[vel.x, vel.y, vel.z, acc.x, acc.y, acc.z, 1.0])
    }

    fn derivative_onsurface(&self) -> VectorN<f64, U7> {
        let vel = self.vel.to_omega(self.pos, self.pos.omega).vel;
        let mut acc = self.pos.centrifugal() + self.coriolis();

        // make sure that the total vertical acceleration makes the object conform to the curvature
        // of the surface
        let up = surface_normal(&self.pos.pos);
        let v = vel.norm();
        let r = r_curv(&self.pos.pos);
        let acc_up = acc.dot(&up);
        acc += (-v * v / r - acc_up) * up;

        VectorN::<f64, U7>::from_column_slice(&[vel.x, vel.y, vel.z, acc.x, acc.y, acc.z, 1.0])
    }

    pub fn derivative(&self) -> VectorN<f64, U7> {
        match self.state {
            ObjectState::InFlight => self.derivative_inflight(),
            ObjectState::OnSurface => self.derivative_onsurface(),
        }
    }

    fn color(&self) -> [f32; 3] {
        [self.color.0, self.color.1, self.color.2]
    }

    pub fn step(&mut self, integrator: &mut impl Integrator<Self>, dt: f64) {
        self.path.push_back(self.pos);
        if self.path.len() > MAX_PATH_LEN {
            let _ = self.path.pop_front();
        }
        integrator.propagate_in_place(self, Self::derivative, StepSize::Step(dt));

        let pos = self.pos.to_omega(OMEGA);
        let r = pos.pos.norm();
        let lat_r_gc = (pos.pos.y / r).asin();
        let earth_r = earth_radius(lat_r_gc);

        if r < earth_r {
            self.state = ObjectState::OnSurface;
            self.pos.pos *= earth_r / r;
            self.vel.vel *= r / earth_r;
            // cancel the vertical component of the velocity if negative
            let normal = surface_normal(&pos.pos);
            let mut vel = self.vel.to_omega(pos, OMEGA);
            let v_up = vel.vel.dot(&normal);
            if v_up < 0.0 {
                vel.vel -= v_up * normal;
                self.vel = vel.to_omega(self.pos, self.vel.omega);
            }
        }
    }

    pub fn draw(
        &self,
        omega: f64,
        display: &Display,
        target: &mut Frame,
        program: &Program,
        matrix: &Matrix4<f32>,
        draw_parameters: &DrawParameters,
    ) {
        let pos = self.pos.to_omega(omega);
        let uniforms = uniform! {
            matrix: *(matrix
                        .prepend_translation(
                            &Vector3::new(
                                pos.pos.x as f32,
                                pos.pos.y as f32,
                                pos.pos.z as f32
                            ))
                        .prepend_scaling(300e3)).as_ref(),
            color: self.color(),
        };

        let sphere = Mesh::solid_sphere(display);
        sphere.draw(target, program, &uniforms, draw_parameters);

        let uniforms = uniform! {
            matrix: *matrix.as_ref(),
            color: self.color(),
        };

        let vertex_buffer = VertexBuffer::new(
            display,
            &self
                .path
                .iter()
                .map(|pos| {
                    let pos = pos.to_omega(omega);
                    Vertex {
                        position: [pos.pos.x as f32, pos.pos.y as f32, pos.pos.z as f32],
                    }
                })
                .collect::<Vec<_>>(),
        )
        .unwrap();
        let index_buffer = index::NoIndices(index::PrimitiveType::LineStrip);

        target
            .draw(
                &vertex_buffer,
                &index_buffer,
                program,
                &uniforms,
                draw_parameters,
            )
            .unwrap();

        let vertex_buffer = VertexBuffer::new(display, {
            let pos = self.pos.to_omega(omega);
            let mut vel = self.vel.to_omega(pos, omega);
            vel.vel /= vel.vel.norm();
            vel.vel *= 1e6;
            &[
                Vertex {
                    position: [pos.pos.x as f32, pos.pos.y as f32, pos.pos.z as f32],
                },
                Vertex {
                    position: [
                        (pos.pos.x + vel.vel.x) as f32,
                        (pos.pos.y + vel.vel.y) as f32,
                        (pos.pos.z + vel.vel.z) as f32,
                    ],
                },
            ]
        })
        .unwrap();

        target
            .draw(
                &vertex_buffer,
                &index_buffer,
                program,
                &uniforms,
                draw_parameters,
            )
            .unwrap();
    }
}

impl State for Object {
    type Derivative = VectorN<f64, U7>;

    fn shift_in_place(&mut self, dir: &VectorN<f64, U7>, amount: f64) {
        let shift = dir * amount;
        let vel = Vector3::from_column_slice(&shift.as_ref()[0..3]);
        let acc = Vector3::from_column_slice(&shift.as_ref()[3..6]);
        self.pos.pos += vel;
        self.vel.vel += acc;
        self.pos.t += shift[6];
    }
}
