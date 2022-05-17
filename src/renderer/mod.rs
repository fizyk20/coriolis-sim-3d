mod cubemap;
mod mesh;

use glium::{implement_vertex, uniform, Display, Frame, Program, Surface};
use nalgebra::{Matrix4, Vector3};

use crate::{
    simulation::{OMEGA, R_EQU, R_POL},
    State,
};
use cubemap::Cubemap;
pub use mesh::Mesh;

const VERTEX_SHADER_SRC: &'static str = r#"
    #version 140

    in vec3 position;

    uniform mat4 matrix;
    uniform vec3 color;
    out vec3 in_color;

    void main() {
        gl_Position = matrix * vec4(position, 1.0);
        in_color = color;
    }
"#;

const FRAGMENT_SHADER_SRC: &'static str = r#"
    #version 140

    in vec3 in_color;
    out vec4 color;

    void main() {
        color = vec4(in_color, 1.0);
    }
"#;

#[derive(Debug, Clone, Copy)]
pub struct Vertex {
    pub position: [f32; 3],
}

implement_vertex!(Vertex, position);

pub struct Renderer {
    program: Program,
    earth_solid_sphere: Mesh,
    sphere: Mesh,
    cubemap: Cubemap,
}

fn galactic_matrix() -> Matrix4<f32> {
    let center_dec = (-29.0f32 - 28.1 / 3600.0).to_radians();
    let center_ra = (15.0f32 * (17.0 + 45.0 / 60.0 + 40.0 / 3600.0)).to_radians();
    let pole_dec = (27.0f32 + 7.0 / 60.0 + 42.0 / 3600.0).to_radians();
    let pole_ra = (15.0f32 * (12.0 + 51.0 / 60.0 + 26.0 / 3600.0)).to_radians();

    let pos_z = -Vector3::new(
        center_dec.cos() * center_ra.sin(),
        center_dec.sin(),
        center_dec.cos() * center_ra.cos(),
    );
    let pos_y = -Vector3::new(
        pole_dec.cos() * pole_ra.sin(),
        pole_dec.sin(),
        pole_dec.cos() * pole_ra.cos(),
    );
    let pos_x = pos_y.cross(&pos_z);

    let mut matrix = Matrix4::<f32>::identity();

    for i in 0..3 {
        matrix[i] = pos_x[i];
        matrix[4 + i] = pos_y[i];
        matrix[8 + i] = pos_z[i];
    }

    matrix //.transpose()
}

impl Renderer {
    pub fn new(display: &Display) -> Self {
        Renderer {
            program: Program::from_source(display, VERTEX_SHADER_SRC, FRAGMENT_SHADER_SRC, None)
                .unwrap(),
            earth_solid_sphere: Mesh::solid_sphere(display, 120, 240),
            sphere: Mesh::ellipsoid(display),
            cubemap: Cubemap::new(display),
        }
    }

    pub fn draw(&mut self, display: &Display, target: &mut Frame, state: &State) {
        target.clear_color(0.0, 0.0, 0.02, 1.0);
        target.clear_depth(1.0);

        let (width, height) = target.get_dimensions();
        let aspect = width as f32 / height as f32;

        let omega = OMEGA * state.omega;
        // how much has Earth rotated since t=0
        let earth_ang = (OMEGA - omega) * state.t;
        // how much has the frame rotated with respect to the sky
        let skybox_ang = -omega * state.t;

        let dist = state.distance;
        let lat = state.lat;
        let lon = state.lon;

        let camera_ang = state.ang - omega * state.t;

        let perspective = Matrix4::new_perspective(aspect, 45.0_f32.to_radians(), 1000.0, 1e9);
        let view_rot = Matrix4::new_rotation(Vector3::new(lat as f32, 0.0, 0.0))
            * Matrix4::new_rotation(Vector3::new(0.0, -lon - camera_ang as f32, 0.0));
        let view_trans = Matrix4::new_translation(&Vector3::new(0.0, 0.0, -dist));
        let matrix = perspective * view_trans * view_rot;

        let earth_rotation = Matrix4::new_rotation(Vector3::new(0.0, earth_ang as f32, 0.0));
        let skybox_rotation = Matrix4::new_rotation(Vector3::new(0.0, skybox_ang as f32, 0.0));

        let galactic_pole_rot = galactic_matrix();

        let draw_parameters = glium::DrawParameters {
            depth: glium::draw_parameters::Depth {
                test: glium::draw_parameters::DepthTest::IfLessOrEqual,
                write: true,
                ..Default::default()
            },
            line_width: Some(4.0),
            ..Default::default()
        };

        self.cubemap.draw(
            target,
            &(perspective * view_rot * skybox_rotation * galactic_pole_rot),
            &draw_parameters,
        );

        let scaling = Matrix4::new_nonuniform_scaling(&Vector3::new(
            (R_EQU * 0.995) as f32,
            (R_POL * 0.995) as f32,
            (R_EQU * 0.995) as f32,
        ));
        let uniforms = uniform! {
            matrix: *(matrix * earth_rotation * scaling).as_ref(),
            color: [0.1_f32, 0.25, 0.1],
        };

        self.earth_solid_sphere
            .draw(target, &self.program, &uniforms, &draw_parameters);

        let uniforms = uniform! {
            matrix: *(matrix * earth_rotation).as_ref(),
            color: [0.4_f32, 1.0, 0.4],
        };

        self.sphere
            .draw(target, &self.program, &uniforms, &draw_parameters);

        let obj_ang = 0.0;
        let obj_rotation = Matrix4::new_rotation(Vector3::new(0.0, obj_ang as f32, 0.0));

        for obj in &state.objects {
            obj.draw(
                omega,
                display,
                target,
                &self.program,
                &(matrix * obj_rotation),
                &glium::DrawParameters {
                    line_width: Some(6.0),
                    ..draw_parameters.clone()
                },
            );
        }
    }
}
