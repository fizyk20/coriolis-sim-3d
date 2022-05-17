use std::{collections::VecDeque, rc::Rc};

use glium::uniform;
use nalgebra::{base::dimension::U7, Matrix4, Vector3, VectorN};
use numeric_algs::{
    integration::{Integrator, StepSize},
    State,
};

use super::{earth_radius, lat_lon_elev_to_vec3, r_curv, surface_normal, GM, OMEGA};
use crate::renderer::Painter;

#[derive(Debug, Clone, Copy)]
pub struct Position {
    pub t: f64,
    pub pos: Vector3<f64>,
    // angular velocity of the frame of reference
    pub omega: f64,
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

    fn coriolis(&self) -> Vector3<f64> {
        let omega_v = Vector3::new(0.0, self.omega, 0.0);
        -2.0 * omega_v.cross(&self.vel)
    }
}

const MAX_PATH_LEN: usize = 5000;

#[derive(Debug, Clone, Copy)]
enum ObjectState {
    InFlight,
    OnSurface,
}

#[derive(Clone)]
pub struct Object {
    pub pos: Position,
    pub vel: Velocity,
    color: (f32, f32, f32),
    radius: f32,
    path: VecDeque<Position>,
    gm: f64,
    drag_coeff: f64,
    friction: f64,
    attractor: Option<Rc<Box<dyn Fn(Position) -> Vector3<f64>>>>,
    state: ObjectState,
}

impl Object {
    pub fn new(pos: Position, vel: Velocity) -> Self {
        Self {
            pos,
            vel,
            color: (1.0, 0.0, 0.0),
            radius: 200e3,
            path: VecDeque::new(),
            gm: GM,
            drag_coeff: 0.0,
            friction: 0.0,
            attractor: None,
            state: ObjectState::InFlight,
        }
    }

    pub fn with_color(self, r: f32, g: f32, b: f32) -> Self {
        Self {
            color: (r, g, b),
            ..self
        }
    }

    pub fn with_radius(self, radius: f32) -> Self {
        Self { radius, ..self }
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

    pub fn with_friction(self, friction: f64) -> Self {
        Self { friction, ..self }
    }

    pub fn as_pendulum(self, coeff: f64) -> Self {
        let pos0 = self.pos;
        let boxed_closure: Box<dyn Fn(Position) -> Vector3<f64>> =
            Box::new(move |pos: Position| coeff * (pos0.to_omega(pos.omega).pos - pos.pos));
        let attractor = Rc::new(boxed_closure);
        Self {
            attractor: Some(attractor),
            ..self
        }
    }

    pub fn with_attractor(self, attractor: Box<dyn Fn(Position) -> Vector3<f64>>) -> Self {
        Self {
            attractor: Some(Rc::new(attractor)),
            ..self
        }
    }

    pub fn time(&self) -> f64 {
        self.pos.t
    }

    fn derivative_inflight(&self) -> VectorN<f64, U7> {
        let vel = self.vel.to_omega(self.pos, self.pos.omega);
        let acc = self.pos.grav(self.gm) + self.pos.centrifugal() + vel.coriolis();
        let vel = vel.vel;

        VectorN::<f64, U7>::from_column_slice(&[vel.x, vel.y, vel.z, acc.x, acc.y, acc.z, 1.0])
    }

    fn friction(&self) -> Vector3<f64> {
        let o = OMEGA - self.pos.omega;
        let vel = self.vel.to_omega(self.pos, self.pos.omega).vel;
        let surf_vel = Vector3::new(o * self.pos.pos.z, 0.0, -o * self.pos.pos.x);
        self.friction * (surf_vel - vel)
    }

    fn attraction_force(&self) -> Vector3<f64> {
        if let Some(attractor) = self.attractor.as_ref() {
            attractor(self.pos)
        } else {
            Vector3::zeros()
        }
    }

    fn derivative_onsurface(&self) -> VectorN<f64, U7> {
        let vel = self.vel.to_omega(self.pos, self.pos.omega);
        // gravity, centrifugal and reaction from the ground should yield a net force equal to the
        // centripetal force according to the local radius of curvature of the surface
        let mut acc = vel.coriolis() + self.friction() + self.attraction_force();
        let vel = vel.vel;

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
        painter: &mut Painter<'_, '_, '_, '_, '_>,
        omega: f64,
        matrix: &Matrix4<f32>,
        draw_velocity: bool,
        draw_forces: bool,
        vel_scale: f64,
        force_scale: f64,
    ) {
        let pos = self.pos.to_omega(omega);
        let matrix_trans = matrix.prepend_translation(&Vector3::new(
            pos.pos.x as f32,
            pos.pos.y as f32,
            pos.pos.z as f32,
        ));
        let uniforms = uniform! {
            matrix: *(matrix_trans.prepend_scaling(self.radius)).as_ref(),
            color: self.color(),
        };

        painter.solid_sphere(&uniforms);

        let uniforms = uniform! {
            matrix: *matrix.as_ref(),
            color: self.color(),
        };

        painter.path(
            &uniforms,
            &self
                .path
                .iter()
                .map(|pos| {
                    let pos = pos.to_omega(omega);
                    Vector3::new(pos.pos.x as f32, pos.pos.y as f32, pos.pos.z as f32)
                })
                .collect::<Vec<_>>(),
        );

        if draw_velocity {
            // draw the velocity direction
            let pos = self.pos.to_omega(omega);
            let mut vel = self.vel.to_omega(pos, omega);
            vel.vel *= vel_scale;

            self.draw_vector(vel.vel, painter, &matrix_trans, self.color());
        }

        if draw_forces {
            let pos = self.pos.to_omega(omega);
            let vel = self.vel.to_omega(self.pos, omega);
            let grav = pos.grav(self.gm) * force_scale;
            let centri = pos.centrifugal() * force_scale;
            let coriolis = vel.coriolis() * force_scale;

            self.draw_vector(grav, painter, &matrix_trans, [0.5, 0.5, 0.0]);
            self.draw_vector(centri, painter, &matrix_trans, [0.3, 1.0, 0.3]);
            self.draw_vector(coriolis, painter, &matrix_trans, [0.0, 1.0, 1.0]);
        }
    }

    fn draw_vector(
        &self,
        vec: Vector3<f64>,
        painter: &mut Painter<'_, '_, '_, '_, '_>,
        matrix: &Matrix4<f32>,
        color: [f32; 3],
    ) {
        let len = vec.norm();
        let ang_x = (vec.y / len).asin() as f32;
        let ang_y = vec.x.atan2(vec.z) as f32;

        let rot_x = Matrix4::new_rotation(Vector3::new(-ang_x, 0.0, 0.0));
        let rot_y = Matrix4::new_rotation(Vector3::new(0.0, ang_y, 0.0));
        let scale = Matrix4::new_nonuniform_scaling(&Vector3::new(
            1.0,
            1.0,
            len as f32 / self.radius / 8.0,
        ));
        let scale2 = Matrix4::new_scaling(self.radius * 8.0);

        let matrix = matrix * rot_y * rot_x * scale * scale2;

        let uniforms = uniform! { matrix: *matrix.as_ref(), color: color };
        painter.arrow(&uniforms);
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
