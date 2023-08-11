use std::{collections::VecDeque, iter, rc::Rc};

use glium::uniform;
use nalgebra::{Matrix4, SVector, Vector3};
use numeric_algs::{
    integration::{Integrator, StepSize},
    State,
};

use super::{
    air_density, earth_radius, pos_to_lat_lon_elev, r_curv, surface_normal, Position, Velocity, GM,
    OMEGA,
};
use crate::{renderer::Painter, state::RenderSettings};

const MAX_PATH_LEN: usize = 50000;

#[derive(Debug, Clone, Copy)]
enum ObjectState {
    FreeFlight,
    ConstantAltitude(f64),
}

#[derive(Debug, Clone, Copy)]
struct SimState {
    pos: Position,
    vel: Velocity,
}

impl SimState {
    fn coriolis_counteraction(&self) -> Vector3<f64> {
        let pos = self.pos.to_omega(OMEGA);
        let vel = self.vel.to_omega(self.pos, OMEGA);

        let up = surface_normal(&pos.pos());
        let coriolis = vel.coriolis();
        let horizontal_coriolis = coriolis - up * up.dot(&coriolis);

        pos.dir_to_omega(-horizontal_coriolis, self.pos.omega())
    }

    fn friction(&self, friction: f64) -> Vector3<f64> {
        let o = OMEGA - self.pos.omega();
        let vel = self.vel.to_omega(self.pos, self.pos.omega()).vel();
        let surf_vel = Vector3::new(o * self.pos.pos().z, 0.0, -o * self.pos.pos().x);
        friction * (surf_vel - vel)
    }

    fn drag(&self, drag_coeff: f64) -> Vector3<f64> {
        let o = OMEGA - self.pos.omega();
        let (_, _, elev) = pos_to_lat_lon_elev(self.pos.to_omega(OMEGA).pos());
        let density = air_density(elev);
        let vel = self.vel.to_omega(self.pos, self.pos.omega()).vel();
        let surf_vel = Vector3::new(o * self.pos.pos().z, 0.0, -o * self.pos.pos().x);
        let vel_diff = surf_vel - vel;
        drag_coeff * density * vel_diff.norm() * vel_diff
    }
}

#[derive(Clone)]
pub struct Object {
    sim_state: SimState,
    color: (f32, f32, f32),
    radius: f32,
    path: VecDeque<SimState>,
    gm: f64,
    drag_coeff: f64,
    friction: f64,
    attractor: Option<Rc<Box<dyn Fn(Position) -> Vector3<f64>>>>,
    counteract_coriolis: bool,
    state: ObjectState,
}

impl Object {
    pub fn new(pos: Position, vel: Velocity) -> Self {
        Self {
            sim_state: SimState { pos, vel },
            color: (1.0, 0.0, 0.0),
            radius: 200e3,
            path: VecDeque::new(),
            gm: GM,
            drag_coeff: 0.0,
            friction: 0.0,
            attractor: None,
            counteract_coriolis: false,
            state: ObjectState::FreeFlight,
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

    pub fn with_const_alt(self, alt: f64) -> Self {
        Self {
            state: ObjectState::ConstantAltitude(alt),
            ..self
        }
    }

    pub fn as_pendulum(self, coeff: f64) -> Self {
        let pos0 = self.sim_state.pos;
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

    pub fn counteract_coriolis(self, counteract_coriolis: bool) -> Self {
        Self {
            counteract_coriolis,
            ..self
        }
    }

    pub fn time(&self) -> f64 {
        self.sim_state.pos.t()
    }

    pub fn pos(&self) -> Position {
        self.sim_state.pos
    }

    pub fn vel(&self) -> Velocity {
        self.sim_state.vel
    }

    fn derivative_inflight(&self) -> SVector<f64, 7> {
        let drag = self.sim_state.drag(self.drag_coeff);
        let vel = self.vel().to_omega(self.pos(), self.pos().omega());
        let acc = self.pos().grav(self.gm) + self.pos().centrifugal() + vel.coriolis() + drag;
        let vel = vel.vel();

        SVector::<f64, 7>::from_column_slice(&[vel.x, vel.y, vel.z, acc.x, acc.y, acc.z, 1.0])
    }

    fn attraction_force(&self) -> Vector3<f64> {
        if let Some(attractor) = self.attractor.as_ref() {
            attractor(self.sim_state.pos)
        } else {
            Vector3::zeros()
        }
    }

    fn derivative_const_alt(&self, alt: f64) -> SVector<f64, 7> {
        let vel = self.vel().to_omega(self.pos(), self.pos().omega());
        let coriolis_counteraction = if self.counteract_coriolis {
            self.sim_state.coriolis_counteraction()
        } else {
            Vector3::new(0.0, 0.0, 0.0)
        };
        // gravity, centrifugal and reaction from the ground should yield a net force equal to the
        // centripetal force according to the local radius of curvature of the surface
        let mut acc = vel.coriolis()
            + self.sim_state.friction(self.friction)
            + self.sim_state.drag(self.drag_coeff)
            + self.attraction_force()
            + coriolis_counteraction;
        let vel = vel.vel();

        // make sure that the total vertical acceleration makes the object conform to the curvature
        // of the surface of constant altitude
        let up = surface_normal(&self.pos().pos());
        let v = vel.norm();
        let r = r_curv(&self.pos().pos());
        let acc_up = acc.dot(&up);
        acc += (-v * v / (r + alt) - acc_up) * up;

        SVector::<f64, 7>::from_column_slice(&[vel.x, vel.y, vel.z, acc.x, acc.y, acc.z, 1.0])
    }

    pub fn derivative(&self) -> SVector<f64, 7> {
        match self.state {
            ObjectState::FreeFlight => self.derivative_inflight(),
            ObjectState::ConstantAltitude(alt) => self.derivative_const_alt(alt),
        }
    }

    fn color(&self) -> [f32; 3] {
        [self.color.0, self.color.1, self.color.2]
    }

    pub fn step(&mut self, integrator: &mut impl Integrator<Self>, dt: f64) {
        self.path.push_back(self.sim_state);
        if self.path.len() > MAX_PATH_LEN {
            let _ = self.path.pop_front();
        }
        integrator.propagate_in_place(self, Self::derivative, StepSize::Step(dt));

        let pos = self.pos().to_omega(OMEGA);
        let r = pos.pos().norm();
        let lat_r_gc = (pos.pos().y / r).asin();
        let earth_r = earth_radius(lat_r_gc);

        let maybe_target_r = match self.state {
            ObjectState::FreeFlight if r < earth_r => Some(earth_r),
            ObjectState::ConstantAltitude(alt) => Some(earth_r + alt),
            _ => None,
        };

        if let Some(target_r) = maybe_target_r {
            self.state = ObjectState::ConstantAltitude(target_r - earth_r);

            let mut vel = self.vel().to_omega(self.pos(), 0.0);
            self.sim_state.pos.mul(target_r / r);
            vel.mul(r / target_r);
            self.sim_state.vel = vel.to_omega(self.pos(), self.pos().omega());
            // cancel the vertical component of the velocity if negative
            let normal = surface_normal(&pos.pos());
            let mut vel = self.vel().to_omega(pos, OMEGA);
            let v_up = vel.vel().dot(&normal);
            if v_up < 0.0 {
                vel.increase(-v_up * normal);
                self.sim_state.vel = vel.to_omega(self.pos(), self.vel().omega());
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
        let states: Vec<_> = self
            .path
            .iter()
            .copied()
            .chain(iter::once(self.sim_state))
            .enumerate()
            .take_while(|(i, state)| *i == 0 || state.pos.t() < render_settings.max_t)
            .map(|(_, state)| state)
            .collect();

        let state = states.last().unwrap();
        let pos = state.pos.to_omega(omega);
        let vel = state.vel.to_omega(pos, omega);

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
            &states
                .iter()
                .map(|state| {
                    let pos = state.pos.to_omega(omega);
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

            if self.counteract_coriolis {
                let force = state
                    .pos
                    .dir_to_omega(state.coriolis_counteraction(), omega)
                    * render_settings.force_scale;
                self.draw_vector(force, painter, &matrix_trans, [0.0, 0.0, 0.9]);
            }
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

    pub fn status(&self, omega: f64, render_settings: &RenderSettings) -> Vec<String> {
        let state = if render_settings.max_t < self.time() {
            self.path
                .iter()
                .rev()
                .skip_while(|state| state.pos.t() > render_settings.max_t)
                .next()
                .cloned()
                .unwrap_or_else(|| self.sim_state)
        } else {
            self.sim_state
        };

        let pos_rot = state.pos.to_omega(OMEGA);
        let (lat, lon, elev) = pos_to_lat_lon_elev(pos_rot.pos());

        let pos_s = format!("Position: {:4.2}°, {:4.2}°, {:7.1}", lat, lon, elev);
        let vel_o = state.vel.to_omega(state.pos, omega);
        let vel_s = format!("Speed: {:4.1} m/s", vel_o.vel().norm());

        let mut status = vec![pos_s, vel_s];

        if self.counteract_coriolis {
            let force = state.coriolis_counteraction();
            let grav_plus_cfg = state.pos.grav(self.gm) + state.pos.centrifugal();
            let grav_plus_cfg_mag = grav_plus_cfg.norm();
            let counteraction_mag = force.norm();
            let right = grav_plus_cfg.cross(&state.vel.vel());
            let right = right.dot(&force) > 0.0;
            let ang = (counteraction_mag / grav_plus_cfg_mag).atan().to_degrees();
            let tilt_s = format!(
                "Coriolis correction tilt: {:3.2}° {}",
                ang,
                if right { "to the right" } else { "to the left" }
            );
            status.push(tilt_s);
        }

        status
    }
}

impl State for Object {
    type Derivative = SVector<f64, 7>;

    fn shift_in_place(&mut self, dir: &SVector<f64, 7>, amount: f64) {
        let shift = dir * amount;
        let vel = Vector3::from_column_slice(&shift.as_ref()[0..3]);
        let acc = Vector3::from_column_slice(&shift.as_ref()[3..6]);
        self.sim_state.pos.increase(vel);
        self.sim_state.vel.increase(acc);
        self.sim_state.pos.increase_time(shift[6]);
    }
}
