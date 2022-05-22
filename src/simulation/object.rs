use std::{collections::VecDeque, iter, rc::Rc};

use glium::uniform;
use nalgebra::{base::dimension::U7, Matrix4, Vector3, VectorN};
use numeric_algs::{
    integration::{Integrator, StepSize},
    State,
};

use super::{earth_radius, r_curv, surface_normal, Position, Velocity, GM, OMEGA};
use crate::{renderer::Painter, state::RenderSettings};

const MAX_PATH_LEN: usize = 50000;

#[derive(Debug, Clone, Copy)]
enum ObjectState {
    InFlight,
    OnSurface,
}

#[derive(Clone)]
pub struct Object {
    pos: Position,
    vel: Velocity,
    color: (f32, f32, f32),
    radius: f32,
    path: VecDeque<(Position, Velocity)>,
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
            Box::new(move |pos: Position| coeff * (pos0.to_omega(pos.omega()).pos() - pos.pos()));
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
        self.pos.t()
    }

    pub fn pos(&self) -> Position {
        self.pos
    }

    pub fn vel(&self) -> Velocity {
        self.vel
    }

    fn derivative_inflight(&self) -> VectorN<f64, U7> {
        let vel = self.vel.to_omega(self.pos, self.pos.omega());
        let acc = self.pos.grav(self.gm) + self.pos.centrifugal() + vel.coriolis();
        let vel = vel.vel();

        VectorN::<f64, U7>::from_column_slice(&[vel.x, vel.y, vel.z, acc.x, acc.y, acc.z, 1.0])
    }

    fn friction(&self) -> Vector3<f64> {
        let o = OMEGA - self.pos.omega();
        let vel = self.vel.to_omega(self.pos, self.pos.omega()).vel();
        let surf_vel = Vector3::new(o * self.pos.pos().z, 0.0, -o * self.pos.pos().x);
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
        let vel = self.vel.to_omega(self.pos, self.pos.omega());
        // gravity, centrifugal and reaction from the ground should yield a net force equal to the
        // centripetal force according to the local radius of curvature of the surface
        let mut acc = vel.coriolis() + self.friction() + self.attraction_force();
        let vel = vel.vel();

        // make sure that the total vertical acceleration makes the object conform to the curvature
        // of the surface
        let up = surface_normal(&self.pos.pos());
        let v = vel.norm();
        let r = r_curv(&self.pos.pos());
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
        self.path.push_back((self.pos, self.vel));
        if self.path.len() > MAX_PATH_LEN {
            let _ = self.path.pop_front();
        }
        integrator.propagate_in_place(self, Self::derivative, StepSize::Step(dt));

        let pos = self.pos.to_omega(OMEGA);
        let r = pos.pos().norm();
        let lat_r_gc = (pos.pos().y / r).asin();
        let earth_r = earth_radius(lat_r_gc);

        if r < earth_r {
            self.state = ObjectState::OnSurface;

            let mut vel = self.vel.to_omega(self.pos, 0.0);
            self.pos.mul(earth_r / r);
            vel.mul(r / earth_r);
            self.vel = vel.to_omega(self.pos, self.pos.omega());
            // cancel the vertical component of the velocity if negative
            let normal = surface_normal(&pos.pos());
            let mut vel = self.vel.to_omega(pos, OMEGA);
            let v_up = vel.vel().dot(&normal);
            if v_up < 0.0 {
                vel.increase(-v_up * normal);
                self.vel = vel.to_omega(self.pos, self.vel.omega());
            }
        }
    }

    pub fn draw(
        &self,
        painter: &mut Painter<'_, '_, '_, '_, '_>,
        omega: f64,
        matrix: &Matrix4<f32>,
        render_settings: &RenderSettings,
    ) {
        let positions: Vec<_> = self
            .path
            .iter()
            .copied()
            .chain(iter::once((self.pos, self.vel)))
            .enumerate()
            .take_while(|(i, pos)| *i == 0 || pos.0.t() < render_settings.max_t)
            .map(|(_, pos)| pos)
            .collect();

        let pos = positions.last().unwrap().0.to_omega(omega);
        let vel = positions.last().unwrap().1.to_omega(pos, omega);

        let matrix_trans = matrix.prepend_translation(&Vector3::new(
            pos.pos().x as f32,
            pos.pos().y as f32,
            pos.pos().z as f32,
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
            &positions
                .iter()
                .map(|pos| {
                    let pos = pos.0.to_omega(omega);
                    Vector3::new(pos.pos().x as f32, pos.pos().y as f32, pos.pos().z as f32)
                })
                .collect::<Vec<_>>(),
        );

        if render_settings.draw_velocities {
            // draw the velocity direction
            let vel = vel.vel() * render_settings.vel_scale;

            self.draw_vector(vel, painter, &matrix_trans, self.color());
        }

        if render_settings.draw_forces {
            let grav = pos.grav(self.gm) * render_settings.force_scale;
            let centri = pos.centrifugal() * render_settings.force_scale;
            let coriolis = vel.coriolis() * render_settings.force_scale;

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
        self.pos.increase(vel);
        self.vel.increase(acc);
        self.pos.increase_time(shift[6]);
    }
}
